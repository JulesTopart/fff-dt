import os
import json
import signal
import math
import numpy as np
import cv2
import zmq
import gymnasium as gym
from gymnasium import spaces

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecMonitor
from stable_baselines3.common.vec_env import VecTransposeImage  # HWC -> CHW for CnnPolicy

# ---------------------------
# Utilities
# ---------------------------

def create_difference_overlay(current, goal, threshold=128):
    if current.ndim == 3:
        current_gray = current[:, :, 0]
    else:
        current_gray = current
    if goal.ndim == 3:
        goal_gray = goal[:, :, 0]
    else:
        goal_gray = goal
    current_mask = current_gray >= threshold
    goal_mask = goal_gray >= threshold
    overlay = np.zeros((*current_gray.shape, 3), dtype=np.uint8)
    overlay[(current_mask & goal_mask)] = (0, 255, 0)
    overlay[(current_mask & ~goal_mask)] = (0, 0, 255)
    overlay[(~current_mask & goal_mask)] = (255, 0, 0)
    return overlay


def make_coord_grids_01(h, w):
    xs = np.linspace(0.0, 1.0, w, dtype=np.float32)
    ys = np.linspace(0.0, 1.0, h, dtype=np.float32)
    xg, yg = np.meshgrid(xs, ys)  # (h,w)
    return xg, yg




# ---------------------------
# Environment
# ---------------------------

class AdditiveManufacturingEnv(gym.Env):
    """
    Flow-only control: action -> extrusion (e_flow) in [0, 5].
    Robot motion comes from the sim; the policy learns WHEN to paint.

    Observation (uint8, HWC):
      ch0: current_bw (0/255)
      ch1: goal_bw    (0/255)
      ch2: remaining  (goal & !current) (0/255)
      ch3: x_grid01   ([0,255])
      ch4: y_grid01   ([0,255])
    """
    metadata = {"render_modes": ["human"]}

    def __init__(self,
                 zmq_addr: str = "tcp://localhost:5555",
                 img_hw=(128, 128),
                 verbose: bool = False):
        super().__init__()
        self.verbose = verbose

        # Image geometry
        self.H, self.W = img_hw
        self.C = 7  # current, goal, remaining, x, y, gauss, brush

        # Printer / world-to-image
        self.nozzle_mm = 25.0                       # diameter
        self.offset_mm = (-50, 0)                  # (x,y) offset
        self.scale_px_per_mm = 0.65                # pixels per mm
        self.offset_px = (self.offset_mm[0] * self.scale_px_per_mm,
                          self.offset_mm[1] * self.scale_px_per_mm)

        # ZMQ
        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.REQ)
        self.sock.connect(zmq_addr)


        self.curiculum = 0

        # RL bookkeeping
        self.max_steps = None
        self.episode_count = 0
        self.step_count = 0

        # Precompute positional channels in [0,255] uint8
        xg01, yg01 = make_coord_grids_01(self.H, self.W)
        self.x_grid_u8 = np.round(xg01 * 255.0).astype(np.uint8)
        self.y_grid_u8 = np.round(yg01 * 255.0).astype(np.uint8)

        # Last state for delta rewards
        self.last_current_mask = None
        self.last_nozzle_px = (0, 0)

        # brush footprint parameters in pixels
        self.brush_radius_px = int((self.nozzle_mm / 2.0) * self.scale_px_per_mm)
        self.gauss_sigma_px  = max(1, int(self.brush_radius_px * 0.75))


        # ---- Spaces ----
        # Action: 1 float in [-1,1] -> e_flow in [0,5]
        #self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        self.action_space = spaces.Discrete(2)

        # Observation: single image Box (H,W,5) uint8
        self.observation_space = spaces.Box(
            low=0, high=255, shape=(self.H, self.W, self.C), dtype=np.uint8
        )

        # Reward weights (tuned for flow gating)
        self.w_progress = 1.0      # reduction in remaining
        self.w_good = 0.5           # newly painted inside target
        self.w_over = 3.0           # newly painted outside target (penalty)
        self.w_gate_on = 0.5        # ON while goal under nozzle
        self.w_gate_off = 0.25      # penalty when OFF while goal under nozzle
        self.w_offtarget_on = 0.5   # penalty when ON while NO goal under nozzle
        self.w_flow_cost = 0.08     # small per-step e_flow cost

    # --- World/image conversions ---
    def _world_to_img(self, x_mm, y_mm):
        x_px = int(x_mm * self.scale_px_per_mm + self.offset_px[0])
        y_px = int(y_mm * self.scale_px_per_mm + self.offset_px[1])
        return x_px, y_px

    # --- Gym API ---
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.episode_count += 1
        self.step_count = 0
        self.last_current_mask = None

        self.sock.send_json({"type": "reset"})
        _ = self.sock.recv()

        obs = self._get_obs()
        return obs, {}

    def setMaxSteps(self, max_steps: int):
        self.max_steps = max_steps

    def _get_obs(self):
        # --- images from sim (uint8 BGR)
        self.sock.send_json({"type": "get_images"})
        cur_buf = self.sock.recv()
        goal_buf = self.sock.recv()
        curr_img = np.frombuffer(cur_buf,  dtype=np.uint8).reshape((self.H, self.W, 3))
        goal_img = np.frombuffer(goal_buf, dtype=np.uint8).reshape((self.H, self.W, 3))

        # --- nozzle position (mm) -> pixels
        self.sock.send_json({"type": "get_nozzle_position"})
        pos_msg = self.sock.recv()
        pos = json.loads(pos_msg)
        nozzle_x_mm = float(np.clip(pos["x"], 0, 300))
        nozzle_y_mm = float(np.clip(pos["y"], 0, 300))
        nx, ny = self._world_to_img(nozzle_x_mm, nozzle_y_mm)
        nx = int(np.clip(nx, 0, self.W - 1))
        ny = int(np.clip(ny, 0, self.H - 1))
        self.last_nozzle_px = (nx, ny)

        # --- base BW masks
        curr_bw = curr_img[:, :, 0]                       # 0..255
        goal_bw = goal_img[:, :, 0]                       # 0..255
        remaining = ((goal_bw >= 128) & (curr_bw < 128)).astype(np.uint8) * 255

        # --- pose channels
        yy, xx = np.ogrid[:self.H, :self.W]
        d2 = (xx - nx) * (xx - nx) + (yy - ny) * (yy - ny)

        gauss = np.exp(-d2 / (2.0 * (self.gauss_sigma_px ** 2) + 1e-8))
        gauss_u8 = np.round(gauss * 255.0).astype(np.uint8)

        brush_u8 = (d2 <= (self.brush_radius_px ** 2)).astype(np.uint8) * 255

        # --- stack to (H,W,7) uint8
        image = np.stack(
            [curr_bw, goal_bw, remaining, self.x_grid_u8, self.y_grid_u8, gauss_u8, brush_u8],
            axis=-1
        ).astype(np.uint8)

        return image

    def step(self, action):
        # --- Action: flow in [0,1]
        a = float(action)
        e_flow = float(np.clip(a * 3.0, 0.0, 3.0))

        # Send action (vx=vy=0 here)
        self.sock.send_json({"type": "step", "xye": [0.0, 0.0, e_flow]})
        _ = self.sock.recv()

        # Gather obs AFTER the step
        obs = self._get_obs()
        self.step_count += 1

        # Build binary masks from obs (uint8)
        curr_bw = obs[:, :, 0]
        goal_bw = obs[:, :, 1]
        current_mask = (curr_bw >= 128)
        goal_mask = (goal_bw >= 128)

        # Newly-painted since last step
        if self.last_current_mask is None:
            painted_new = current_mask
        else:
            painted_new = current_mask & (~self.last_current_mask)

        # Global terms
        good_new = painted_new & goal_mask
        over_new = painted_new & (~goal_mask)

        goal_area = int(np.sum(goal_mask)) + 1e-6
        remaining_prev = int(np.sum(goal_mask & (~(self.last_current_mask if self.last_current_mask is not None else np.zeros_like(current_mask, dtype=bool)))))
        remaining_now  = int(np.sum(goal_mask & (~current_mask)))

        # Progress shaping
        r_progress = self.w_progress * (remaining_prev - remaining_now) / goal_area

        # Coverage/Overspray on new paint
        r_good = self.w_good * (np.sum(good_new) / goal_area)
        r_over = - self.w_over * (np.sum(over_new) / goal_area)

        # Gate-aware local term under the nozzle
        nx, ny = self.last_nozzle_px
        nozzle_radius_px = int((self.nozzle_mm / 2.0) * self.scale_px_per_mm)
        Y, X = np.ogrid[:self.H, :self.W]
        dist = np.sqrt((X - nx) ** 2 + (Y - ny) ** 2)
        area_mask = (dist <= nozzle_radius_px)

        goal_under = np.any(goal_mask & area_mask)

        r_gate = 0.0
        if e_flow > 1e-6:
            # ON: reward if on-goal, penalize if off-target
            r_gate += +self.w_gate_on if goal_under else -self.w_offtarget_on
        else:
            # OFF: small penalty if missing opportunity
            if goal_under:
                r_gate -= self.w_gate_off

        # Small flow cost
        r_flow = - self.w_flow_cost * e_flow

        reward = float(r_progress + r_good + r_over + r_gate + r_flow)

        # Done?
        self.sock.send_json({"type": "is_done"})
        done = self.sock.recv_json().get("done", False)
        if not done and self.max_steps and self.step_count >= self.max_steps:
            done = True

        info = dict(
            r_progress=float(r_progress),
            r_good=float(r_good),
            r_over=float(r_over),
            r_gate=float(r_gate),
            r_flow=float(r_flow),
            goal_under=bool(goal_under),
            remaining=int(remaining_now),
            steps=int(self.step_count),
            e_flow=float(e_flow),
        )

        # Bookkeeping for next step & end-of-episode dumps
        self.last_current_mask = current_mask

        if done:
            # Save quick overlays
            os.makedirs("./episodes", exist_ok=True)
            cv2.imwrite(f"./episodes/episode_{self.episode_count}_goal.png", obs[:, :, 1])
            cv2.imwrite(f"./episodes/episode_{self.episode_count}_current.png", obs[:, :, 0])
            overlay = create_difference_overlay(obs[:, :, 0], obs[:, :, 1])
            cv2.imwrite(f"./episodes/episode_{self.episode_count}_metrics.png", overlay)
            cv2.imwrite(f"./episodes/episode_{self.episode_count}_location.png", obs[:,:,5])

            # Simple curriculum hook
            self.curiculum += 1
            if self.curiculum >= 10:
                self.sock.send_json({"type": "phase"})
                _ = self.sock.recv()
                self.curiculum = 0

        if self.verbose:
            print(f"[STEP {self.step_count}] r={reward:+.3f}  prog={r_progress:+.3f} good={r_good:+.3f} over={r_over:+.3f} gate={r_gate:+.3f} flow={e_flow:.2f}")

        return obs, reward, done, False, info

    def render(self, mode="human"):
        # Optional: add your own windows here
        pass

    def close(self):
        self.sock.close()
        self.ctx.term()

# ---------------------------
# Training launcher
# ---------------------------

def make_env(port, verbose=False):
    def _thunk():
        return AdditiveManufacturingEnv(
            zmq_addr=f"tcp://localhost:{port}",
            img_hw=(128, 128),
            verbose=verbose
        )
    return _thunk

if __name__ == "__main__":
    os.makedirs("./episodes", exist_ok=True)

    # Single env quick check
    # (important: check_env before vectorizing)
    _env_check = AdditiveManufacturingEnv(verbose=False)
    _env_check.setMaxSteps(400)
    check_env(_env_check)
    _env_check.close()

    # ---- Parallel training (spawn N sims on different ports: 5555..5555+N-1) ----
    NUM_ENVS = int(os.environ.get("NUM_ENVS", "1"))
    BASE_PORT = int(os.environ.get("BASE_PORT", "5555"))

    if NUM_ENVS == 1:
        env = DummyVecEnv([make_env(BASE_PORT, verbose=False)])
    else:
        env = SubprocVecEnv([make_env(BASE_PORT + i, verbose=False) for i in range(NUM_ENVS)])

    env = VecMonitor(env)
    # Convert HWC->CHW for CnnPolicy
    env = VecTransposeImage(env)

    checkpoint = "ppo_flow_pos_ch"
    if os.path.exists(checkpoint + ".zip"):
        model = PPO.load(checkpoint, env=env, device="cuda")
        print(f"Resumed from {checkpoint}.zip")
    else:
        model = PPO(
            "CnnPolicy",
            env,
            device="cuda",
            verbose=1,
            tensorboard_log="./tb_ppo_flow_pos",
            # A few stable defaults for image+discrete-ish control
            n_steps=1024,
            batch_size=256,
            learning_rate=3e-4,
            clip_range=0.2,
            ent_coef=0.01,
        )

    def _save_and_exit(*_):
        print("Saving model...")
        model.save(checkpoint)
        env.close()
        exit()

    import signal as _sig
    _sig.signal(_sig.SIGINT, _save_and_exit)
    _sig.signal(_sig.SIGTERM, _save_and_exit)

    model.learn(total_timesteps=2_000_000)
    model.save(checkpoint)
    env.close()
