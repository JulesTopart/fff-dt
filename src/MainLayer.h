#pragma once

#include <Merlin.h>
#include "sim/sim.h"
#include "gcode/gcode_sim.h"

using namespace Merlin;

enum class Machine {
	NEOTECH,
	TOOLHANGER
};

enum class ExperimentType {
	NONE = 0,
	VISCOSITY = 1,          // vary viscosity_a (and viscosity_b = ratio * a)
	FLOWRATE = 2,           // vary settings.flow_override
	SPEED = 3,              // vary print/travel feedrates (print)
	LAYER = 4,              // vary layer height
	LINKED_SPEED_FLOW = 5   // vary print speed and flow together
};

struct ExperimentConfig {
	int repeats = 1;
	float viscosity_b_ratio = 1.0f;     // b = ratio * a
	float fixed_viscosity_a = 100.0f;     // used when viscosity is not the varied param
	float fixed_flow_override = 2.513f;    // used when flowrate isn't varied
	float fixed_print_speed = 20.0f;   // used when speed isn't varied
	float fixed_travel_speed = 300.0f;  // used when speed isn't varied
	float fixed_layer_height = 0.3f;  // used when speed isn't varied

	// Linked S&F tuning
	float layer_height = 5.0;   // print_speed  = bias + gain * v
	float link_speed_gain = 1.0f;   // print_speed  = bias + gain * v
	float link_speed_bias = 0.0f;
	float link_flow_gain = 1.0f;   // flow_override = bias + gain * v
	float link_flow_bias = 0.0f;
	bool auto_start = true;
};

class MainLayer : public Layer3D {
public:
	MainLayer();
	~MainLayer();

	void onAttach() override;
	void onDetach() override;
	void onUpdate(Timestep ts) override;
	void onImGuiRender() override;

	void createGCode();
	void createBuffers();
	void createScene();
	void createShaders();
	void createCamera();

	void syncUniform();

	std::string slice(Mesh_Ptr mesh, glm::vec3 normal, float h);

public:

	void plotIsoSurface();
	void plotCutView();
	void plotProfile();

	void plotXY();
	void plotXZ();
	void plotYZ();
	void plotdepthMap();

private:
	// ----------- Experiment runner helpers -----------
	void startExperiment(ExperimentType type);
	void stopExperiment();
	bool prepareRunForCurrentPlan();          // apply params & (re)create gcode; returns false if finished
	void advanceAfterRun();                   // increment indices & queue next or stop
	void saveRunOutputs(Mesh_Ptr mesh, const std::string& svg);
	std::string currentRunTag() const;        // "mode-<...>_val-<...>_rep-<n>"
	static std::string sanitizeFloat(float v, int precision = 3);
	static std::string expTypeToString(ExperimentType t);
	void saveOutput(bool store_results = true);

private:

	Scene_Ptr scene;
	Renderer renderer;

	Sim sim;
	GCodeSimulator controller;

	/*********** Scene ***********/
	Mesh_Ptr iso_mesh = nullptr;

	Mesh_Ptr obstacle;

	TransformObject_Ptr origin;

	ParticleSystem_Ptr ps;
	ParticleSystem_Ptr bs;

	Mesh_Ptr nozzle;

	Camera profile_camera;
	Shader_Ptr profile_shader;
	FBO_Ptr profile_fbo;
	RBO_Ptr profile_rbo;
	Texture2D_Ptr profile_texture;
	Mesh_Ptr profile;
	glm::vec3 profile_size;


	/*********** Visualization ***********/
	float offsetPlane = 0;
	IsoSurface_Ptr isosurface;
	Texture3D_Ptr volume;

	Texture2D_Ptr texture_debugXZ;
	Texture2D_Ptr texture_debugXY;
	Texture2D_Ptr texture_debugYZ;

	Texture2D_Ptr texture_debug_depth;


	/*********** Shaders ***********/
	Shader_Ptr particle_shader;
	Shader_Ptr isosurface_shader;
	Shader_Ptr bin_shader;

	ComputeShader_Ptr solver;
	ComputeShader_Ptr isoGen;
	ComputeShader_Ptr texPlot;
	ComputeShader_Ptr depthPlot;

	GLsizeiptr last_numParticle = 0;
	bool need_sync = true;

	/*********** Rendering ***********/
	bool use_isosurface = true;
	bool use_2Dplot = false;
	bool use_profile_plot = true;

	/*********** Experiment state ***********/
	ExperimentType exp_type = ExperimentType::VISCOSITY;
	ExperimentConfig exp_cfg;
	bool store_results = false;   // whether to save outputs

	// Uses the top-of-file array_plan values as the sweep plan.
	// We copy them into this vector once on attach so we can show/edit in ImGui without touching the existing globals.
	std::vector<float> plan_values;      
	std::vector<float> results_values;    

	int exp_index = 0;                   // which value in plan_values
	int exp_repeat = 0;                  // 0..repeats-1
	bool exp_running = false;

	// Feedrates that createGCode() will use (instead of hard-coded numbers).
	float print_speed = 1500.0f;
	float travel_speed = 1000.0f;
	float layer_height = 5;

	// Bookkeeping: count completed runs to show a simple progress bar
	int total_runs_planned = 0;
	int total_runs_done = 0;
};
