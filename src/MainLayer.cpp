#include "MainLayer.h"
#include "settings.h"

using namespace Merlin;

#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <filesystem>
#include <chrono>
#include <GLFW/glfw3.h>

#include "slicer/slicer.h"

namespace fs = std::filesystem;

static constexpr int kMaxPlanValues = 30;  // new
float viscosity = 70.0f; //Viscosity of the fluid










struct ViscParams {
	int   iterations;  // solver iterations
	float kV;          // viscosity gain
	float kP;          // pressure gain
};

inline ViscParams paramsForViscosity(double mu_Pa_s) {
	// --- calibration table (your data) ---
	static const double MU[] = { 0.25,  1.0,   5.0,   50.0, 100.0 };
	static const double KV[] = { 10.0, 60.0, 150.0, 200.0, 500.0 };
	static const double KP[] = { 10.0, 10.0,  10.0,  10.0, 100.0 };
	static const int    IT[] = { 10,   15,    20,    30,    30 };
	constexpr int N = 5;

	auto lerp = [](double a, double b, double t) { return a + t * (b - a); };
	if (!(mu_Pa_s > 0.0)) mu_Pa_s = MU[0];

	// clamp to table ends
	if (mu_Pa_s <= MU[0]) return { IT[0], (float)KV[0], (float)KP[0] };
	if (mu_Pa_s >= MU[N - 1]) return { IT[N - 1], (float)KV[N - 1], (float)KP[N - 1] };

	// find segment i s.t. MU[i] <= mu < MU[i+1]
	int i = 0;
	while (i + 1 < N && mu_Pa_s > MU[i + 1]) ++i;

	// interpolate in log(mu) to smooth wide ranges
	const double x0 = std::log10(MU[i]);
	const double x1 = std::log10(MU[i + 1]);
	const double x = std::log10(mu_Pa_s);
	const double t = (x - x0) / (x1 - x0); // 0..1

	// kV, kP: interpolate
	const double kV = lerp(KV[i], KV[i + 1], t);
	const double kP = lerp(KP[i], KP[i + 1], t);

	// iterations: step at the 5↔50 break (use log midpoint)
	const double iter_break = std::sqrt(5.0 * 50.0); // ≈ 15.81 Pa·s
	const int iterations = (mu_Pa_s < iter_break) ? 20 : 30;

	return { iterations, (float)kV, (float)kP };
}



// ==================== Layer lifecycle ====================
MainLayer::MainLayer() {
	camera().setNearPlane(2.0f);
	camera().setFarPlane(800);
	camera().setFOV(45); //Use 90.0f as we are using cubemaps
	camera().setPosition(glm::vec3(0, 0, 150));

	glm::vec3 viewCenter = glm::vec3(0, 0, 0);
	camera().setView(CameraView::Iso, glm::distance(glm::vec3(0), camera().getPosition()), viewCenter);

	glfwSwapInterval(0);

	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_LINE_SMOOTH);
}

MainLayer::~MainLayer() {}

void MainLayer::onAttach() {
	Console::printSeparator();

	Layer3D::onAttach();

	Console::setLevel(ConsoleLevel::_INFO);

	// initial physics
	//settings.viscosity_a = viscosity;
	//settings.viscosity_b = viscosity / 4.0f;
	//settings.artificialViscosityMultiplier = 25 * 0.01f;

	// init feedrates used by createGCode()
	print_speed = 20.0f;
	travel_speed = 200.0f;

	// copy the global array_plan into editable vector
	plan_values = { 0.25f, 1.0f, 5.0f, 50.0f, 100.0f};
	results_values.resize(plan_values.size(), 0.0f);

	createBuffers();
	sim.init();
	createShaders();
	createCamera();
	createScene();
	createGCode();
	controller.reset();
	sim.reset();

}

void MainLayer::onDetach() {
	Layer3D::onDetach();
	sim.stop();
}

void MainLayer::onUpdate(Timestep ts) {
	Layer3D::onUpdate(ts);

	static int material = -1;

	GPU_PROFILE(render_time,
		syncUniform();

		if (sim.isRunning()) {
			plotCutView();
			plotIsoSurface();
		}

		renderer.clear();
		renderer.render(scene, camera());
		renderer.reset();
	)

		need_sync = false;

	if (last_numParticle != settings.numParticles()) need_sync = true;

	if (sim.isRunning()) {
		controller.update(settings.max_timestep);
		if (material != controller.getNozzleMaterial()) {
			material = controller.getNozzleMaterial();

			solver->use();
			solver->setInt("fluid_emitter_color", material);
		}

		nozzle->setPosition(controller.getNozzlePosition());
		sim.setNozzlePosition(controller.getNozzlePosition());
		sim.setFlowrate(controller.getFlowrate());

		sim.step(settings.max_timestep);
		MemoryManager::instance().resetBindings();

		// ------ Finished a single run ------
		if (controller.lastCommandReached() && exp_running) {
			// stop stepping for a moment while we save & queue next run
			sim.stop();
			saveOutput(store_results);
			advanceAfterRun(); // will call prepareRunForCurrentPlan() and (optionally) restart sim
		}
	}
}


void MainLayer::createCamera() {
	profile_fbo = FBO::create(512, 512);

	profile_rbo = RBO::create();
	profile_rbo->reserve(512, 512, GL_DEPTH24_STENCIL8);

	profile_texture = Texture2D::create(512, 512, 4, 8, TextureType::ALBEDO);
	profile_texture->bind();
	profile_texture->setInterpolationMode(GL_LINEAR, GL_LINEAR);
	profile_texture->setRepeatMode(GL_CLAMP_TO_BORDER, GL_CLAMP_TO_BORDER);
	profile_texture->setBorderColor4f(1, 1, 1, 1);
	profile_texture->unbind();

	profile_fbo->bind();
	profile_texture->bind();
	profile_fbo->attachColorTexture(profile_texture);
	profile_rbo->bind();
	profile_fbo->attachDepthStencilRBO(profile_rbo);
	profile_fbo->unbind();
	profile_rbo->unbind();
	profile_texture->unbind();

	profile_camera.setView(CameraView::Front, 100, glm::vec3(0, 0, 0));

	//camera_output(img_res, img_res)
	static float nearPlane = 1;
	static float farPlane = 100;

	profile_camera.setNearPlane(nearPlane);
	profile_camera.setFarPlane(farPlane);

}


void MainLayer::plotProfile() {

	profile_shader->use();
	profile_texture->bind();
	//renderer.renderTo(profile_fbo);
	renderer.activateTarget();
	renderer.clear();
	renderer.render(profile, profile_camera);
	renderer.reset();

	int width = profile_texture->width();
	int height = profile_texture->height();

	std::vector<uint8_t> pixels(width * height * 3);

	profile_texture->bind();
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
	profile_texture->unbind();
}

glm::vec3 _z(glm::vec3& v, float z) {
	return glm::vec3(v.x, v.y, v.z + z);
}

// ==================== GCode: now uses instance feedrates ====================
void MainLayer::createGCode() {
	controller.clear();
	//settings.flow_override = settings.flow_override; // unchanged – explicit to keep intent

	//controller.readFile("./assets/simam.gcode");
	//controller.readFile("./assets/cube.gcode");

	// Use member feedrates set by the experiment runner or UI
	const float travelF = std::max(1.0f, travel_speed);
	const float printF = std::max(1.0f, print_speed);
	const float flow = settings.flow_override;//mm3.s-1

	//Boundary Volume dimensions
	#ifdef BEAD_TEST
		controller.move(glm::vec3(-20, -10, layer_height), 0, travelF); // travel/perimeter start

		controller.move(glm::vec3(-20, -30, layer_height), flow*1.5, printF);  // print
		controller.move(glm::vec3(  0, -30, layer_height), flow, printF);  // print
		controller.move(glm::vec3(  0,  15, layer_height), flow, printF);  // print
		controller.move(glm::vec3(  0,  15, layer_height*3), flow, printF*0.1);  // print
	#elif defined(COIL_TEST)
		controller.move(glm::vec3(0, 0, 30), 0, travelF); // travel/perimeter start
		controller.move(glm::vec3(0, 0, 35), flow, 0.5);  // print
	#elif defined(MULTI_BEAD_TEST)

		// params you can tweak
		const int   layers = 3;
		const int   lines = 5;          // beads per layer
		const float pitch = 5.0f;       // spacing between parallel beads (mm)
		const float x0 = -15;         // left X
		const float x1 = 20.0f;          // right X
		const float y0 = -10.0f;         // first bead Y
		const float primeLen = 20.0f;    // prime distance (mm)

		// 3 layers
		for (int L = 0; L < layers; ++L) {
			const float z = (L + 1) * layer_height;

			// Prime at the start of each layer (same style as your snippet)
			controller.move(glm::vec3(x0, y0 + primeLen, z), 0, travelF); // travel to prime start
			controller.move(glm::vec3(x0, y0, z), flow * 1.5f, printF); // prime

			// Lay down 5 parallel beads, serpentine (→ then ←) to reduce travel
			float y = y0;
			bool toRight = true;

			// ensure we are exactly at the start of the first line
			controller.move(glm::vec3(x0, y, z), 0, travelF);

			for (int i = 0; i < lines; ++i) {
				if (toRight) {
					controller.move(glm::vec3(x1, y, z), flow, printF);  // print →
				}
				else {
					controller.move(glm::vec3(x0, y, z), flow, printF);  // print ←
				}

				// move up to next bead (travel only), flip direction
				if (i < lines - 1) {
					y += pitch;
					const float xStay = toRight ? x1 : x0;
					controller.move(glm::vec3(xStay, y, z), 0, travelF); // shift to next Y without extruding
					toRight = !toRight;
				}
			}
		}

	#endif

	


	controller.reset();
	sim.reset();
}

// ==================== Buffers/Scene/etc. (unchanged) ====================

void MainLayer::createBuffers() {
	auto& memory = MemoryManager::instance();
	//Particle buffers
	memory.createBuffer<glm::vec4>("last_position_buffer", settings.max_pThread);
	memory.createBuffer<glm::vec4>("position_buffer", settings.max_pThread);
	memory.createBuffer<glm::vec4>("predicted_position_buffer", settings.max_pThread);
	memory.createBuffer<glm::vec4>("correction_buffer", settings.max_pThread);
	memory.createBuffer<glm::vec4>("velocity_buffer", settings.max_pThread);
	memory.createBuffer<glm::vec2>("temperature_buffer", settings.max_pThread);
	memory.createBuffer<float>("density_buffer", settings.max_pThread);
	memory.createBuffer<float>("lambda_buffer", settings.max_pThread);
	
	memory.createBuffer<glm::mat4>("stress_buffer", settings.max_pThread);
	memory.createBuffer<glm::mat4>("F_buffer", settings.max_pThread);
	memory.createBuffer<glm::mat4>("L_buffer", settings.max_pThread);
	
	memory.createBuffer<glm::uvec4>("meta_buffer", settings.max_pThread);
	memory.createBuffer<CopyContent>("copy_buffer", settings.max_pThread);

	//Particle Emitter buffers
	memory.createBuffer<glm::vec4>("emitter_position_buffer", 1);

	//Bin Buffer
	memory.createBuffer<Bin>("bin_buffer", settings.bThread);
}


void MainLayer::createScene() {
	renderer.initialize();
	renderer.enableSampleShading();
	renderer.setEnvironmentGradientColor(1.0, 1.0, 1.0);
	renderer.enableEnvironment();
	//renderer.disableShadows();//Use this to disable shadow casting for the nozzle
	renderer.enableShadows(); // or Use this to enable shadow casting for the nozzle instead
	renderer.disableFaceCulling();

	scene = Scene::create("scene");

	auto& memory = MemoryManager::instance();

	/***********************
		Scene decoration
	************************/

	origin = TransformObject::create("origin", 10);
	origin->setPosition(glm::vec3(-settings.bb.x * 0.5, -settings.bb.y * 0.5, 0));

	obstacle = sim.getBunny();
	obstacle->smoothNormals();
	//obstacle->setMaterial("jade");
	obstacle->setMaterial("pearl");

	nozzle = ModelLoader::loadMesh("./assets/models/nozzle.stl");

	shared<PhongMaterial> brass = createShared<PhongMaterial>("brass");
	brass->setAmbient(glm::vec3(0.24725, 0.1995, 0.0745) * 2.0f);
	brass->setDiffuse(glm::vec3(0.75164, 0.60648, 0.22648));
	brass->setSpecular(glm::vec3(0.628281, 0.555802, 0.366065));
	brass->setShininess(0.8);
	brass->setAlphaBlending(0.7);

	nozzle->setMaterial(brass);
	nozzle->scale(4);
	nozzle->applyMeshTransform();
	nozzle->translate(glm::vec3(0,0,-0.6));
	nozzle->applyMeshTransform();
	//nozzle->castShadow(false); //Use this to disable shadow casting for the nozzle

	/***********************
		Setup Textures
	************************/

	texture_debugXZ = Texture2D::create(settings.tex_size.x, settings.tex_size.z, 4, 16);
	texture_debugXZ->setUnit(0);
	texture_debugXY = Texture2D::create(settings.tex_size.x, settings.tex_size.y, 4, 16);
	texture_debugXY->setUnit(0);
	texture_debugYZ = Texture2D::create(settings.tex_size.y, settings.tex_size.z, 4, 16);
	texture_debugYZ->setUnit(0);

	texture_debug_depth = Texture2D::create(settings.tex_size.y, settings.tex_size.z, 4, 16);
	texture_debug_depth->setUnit(0);

	/***********************
		Setup Iso-Surface
	************************/

	volume = Texture3D::create(settings.volume_size.x, settings.volume_size.y, settings.volume_size.z, 4, 16);
	volume->setUnit(0);
	isosurface = IsoSurface::create("isosurface", volume);
	isosurface->mesh()->setShader(isosurface_shader);
	isosurface->mesh()->translate(settings.bb * glm::vec3(0, 0, 0.5));
	isosurface->mesh()->scale(settings.bb * glm::vec3(1.0, 1.0, 0.5));

	/****************************
		Setup Particle System
	*****************************/

	ps = sim.getParticles();
	bs = sim.getBins();

	ps->setShader(particle_shader);
	bs->setShader(bin_shader);

	ps->setPositionBuffer(memory.getBuffer("position_buffer"));


	/********************************************************
						Setup Lights 
		   [optional] -> automated if commented out
	********************************************************/

	shared<DirectionalLight>  dirlight;

	/**/
	dirlight = createShared<DirectionalLight>("light1", glm::vec3(-0.5f, 0.5f, -0.8f));
	dirlight->translate(glm::vec3(-10, 0, 0));
	dirlight->setDiffuse(glm::vec3(1.0, 1.0, 1.0)*2.0f);
	dirlight->setSpecular(glm::vec3(1.0, 1.0, 1.0)*2.0f);
	scene->add(dirlight);
	/**/

	/*
	dirlight = createShared<DirectionalLight>("light2", glm::vec3(0.5f, 0.5f, -0.8f));
	dirlight->translate(glm::vec3(-10, 0, 0));
	dirlight->setDiffuse(glm::vec3(1));
	scene->add(dirlight);
	/**/

	/*
	dirlight = createShared<DirectionalLight>("light3", glm::vec3(0.0f, -0.5f, -0.8f));
	dirlight->translate(glm::vec3(-10, 0, 0));
	dirlight->setDiffuse(glm::vec3(1));
	scene->add(dirlight);
	/**/

	/**/
	shared<AmbientLight> amLight = createShared<AmbientLight>("light4");
	amLight->setAmbient(glm::vec3(0.2));
	scene->add(amLight);
	/**/


	/****************************
		Scene assembly
	*****************************/

	scene->add(ps);
	scene->add(bs);

	//scene->add(obstacle);

	scene->add(nozzle);
	scene->add(isosurface);

	scene->add(origin);
	Mesh_Ptr plane = Primitives::createRectangle(settings.bb.x, settings.bb.y);
	plane->setMaterial("white plastic");

	scene->add(plane);


	Mesh_Ptr ruler = Primitives::createFloor(10, 1);
	ruler->translate(glm::vec3(0, 0, 0.1f));
	scene->add(ruler);

	bs->hide();
	isosurface->hide();
}

void MainLayer::onImGuiRender() {
	//ImGui::ShowDemoWindow();
	ImGui::DockSpaceOverViewport((ImGuiViewport*)0, ImGuiDockNodeFlags_PassthruCentralNode);

	if (ImGui::BeginMainMenuBar()) {
		ImGui::BeginDisabled();
		if (ImGui::BeginMenu("File")) {
			if (ImGui::MenuItem("New Project", "Ctrl+N", false)) {}
			if (ImGui::MenuItem("Open Project", "Ctrl+O", false)) {}
			if (ImGui::BeginMenu("Recent Project")) {
				ImGui::MenuItem("...", "", false);
				ImGui::EndMenu();
			}
			if (ImGui::MenuItem("Save Project", "Ctrl+S", false)) {}
			if (ImGui::MenuItem("Save Project as", "Ctrl+Alt+S", false)) {}

			ImGui::Separator();
			if (ImGui::BeginMenu("Import")) {
				ImGui::MenuItem("Import Mesh", "Ctrl+I", false);
				ImGui::EndMenu();
			}

			if (ImGui::BeginMenu("Export")) {
				if (ImGui::MenuItem("Export G-Code..", "Ctrl+G", false)) {}
				ImGui::MenuItem("Send G-Code..", "Ctrl+Shift+G", false);
				ImGui::MenuItem("Export G-code to SD-Card", "Ctrl+U", false);
				ImGui::Separator();
				ImGui::MenuItem("Export project as STL", "", false);
				ImGui::Separator();
				ImGui::MenuItem("Export Configuration", "", false);
				ImGui::EndMenu();
			}

			ImGui::Separator();
			if (ImGui::BeginMenu("Convert")) {
				ImGui::MenuItem("Convert ASCII G-code to Binary", "", false);
				ImGui::MenuItem("Convert Binary G-code to ASCII", "", false);
				ImGui::EndMenu();
			}

			ImGui::Separator();
			ImGui::MenuItem("Exit", "Alt+F4", false);
			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Edit")) {
			ImGui::MenuItem("Select All", "Ctrl+A", false);
			ImGui::MenuItem("Deselect All", "Esc", false);
			ImGui::Separator();
			ImGui::MenuItem("Delete Selected", "Del", false);
			ImGui::Separator();
			ImGui::MenuItem("Delete All", "Ctrl+Del", false);
			ImGui::MenuItem("Undo", "Ctrl+Z", false);
			ImGui::MenuItem("Redo", "Ctrl+Y", false);
			ImGui::Separator();
			ImGui::MenuItem("Copy", "Ctrl+C", false);
			ImGui::MenuItem("Paste", "Ctrl+V", false);
			ImGui::EndMenu();
		}
		ImGui::EndDisabled();

		if (ImGui::BeginMenu("View")) {
			glm::vec3 viewCenter = glm::vec3(0, 0, 0);

			if (ImGui::MenuItem("Iso", "0", false)) camera().setView(CameraView::Iso, glm::distance(origin->position(), camera().getPosition()), viewCenter);

			ImGui::Separator();
			if (ImGui::MenuItem("Top", "1", false)) camera().setView(CameraView::Top, glm::distance(origin->position(), camera().getPosition()), viewCenter);
			if (ImGui::MenuItem("Bottom", "2", false)) camera().setView(CameraView::Bottom, glm::distance(origin->position(), camera().getPosition()), viewCenter);
			if (ImGui::MenuItem("Front", "3", false)) camera().setView(CameraView::Front, glm::distance(origin->position(), camera().getPosition()), viewCenter);
			if (ImGui::MenuItem("Rear", "4", false)) camera().setView(CameraView::Rear, glm::distance(origin->position(), camera().getPosition()), viewCenter);
			if (ImGui::MenuItem("Left", "5", false)) camera().setView(CameraView::Left, glm::distance(origin->position(), camera().getPosition()), viewCenter);
			if (ImGui::MenuItem("Right", "6", false)) camera().setView(CameraView::Right, glm::distance(origin->position(), camera().getPosition()), viewCenter);

			ImGui::EndMenu();
		}

		ImGui::MenuItem("Configuration", "", false);

		if (ImGui::BeginMenu("Help")) {
			ImGui::MenuItem("About", "", false);
			ImGui::MenuItem("Report an Issue", "", false);
			ImGui::MenuItem("Documentation", "", false);
			ImGui::EndMenu();
		}
		ImGui::EndMainMenuBar();
	}

	// ---------------- General ----------------
	ImGui::Begin("General");
	ImGui::LabelText("FPS", std::to_string(fps()).c_str());
	ImGui::LabelText(std::to_string(settings.numParticles()).c_str(), "particles");
	ImGui::LabelText(std::to_string(settings.bThread).c_str(), "bins");

	if (ImGui::TreeNode("Statistics")) {
		ImGui::LabelText("FPS", std::to_string(fps()).c_str());

		ImGui::Text("Nearest Neighbor Search %.1f ms", nns_time);
		ImGui::Text("Substep solver %.1f ms", solver_substep_time - nns_time);
		ImGui::Text("Jacobi iteration %.1f ms", jacobi_time);
		ImGui::Text("Total physics time %.1f ms", solver_total_time);
		ImGui::Text("Render time %.1f ms", render_time);
		ImGui::Text("Total frame time %.1f ms", total_time);
		ImGui::TreePop();
	}
	ImGui::End();

	// ---------------- Experiment Runner ----------------
	ImGui::Begin("Experiment Runner");

	// ---- Plan input mode ----
	static int plan_mode = 0; // 0 = Manual, 1 = Range
	ImGui::Text("Plan input:");
	ImGui::RadioButton("Manual", &plan_mode, 0); ImGui::SameLine();
	ImGui::RadioButton("Range", &plan_mode, 1);

	// ---- Manual mode: up to 30 values ----
	if (plan_mode == 0) {
		static int manual_count = (int)plan_values.size();
		manual_count = std::clamp(manual_count, 1, kMaxPlanValues);

		ImGui::SetNextItemWidth(120.0f);
		if (ImGui::DragInt("Number of values", &manual_count, 1, 1, kMaxPlanValues)) {
			manual_count = std::clamp(manual_count, 1, kMaxPlanValues);
			// Preserve existing data where possible; extend with last value or 0
			float fill = plan_values.empty() ? 0.0f : plan_values.back();
			plan_values.resize(manual_count, fill);
			results_values.resize(manual_count, 0);
		}

		ImGui::Separator();
		ImGui::Text("Edit values:");
		for (int i = 0; i < (int)plan_values.size(); ++i) {
			ImGui::PushID(i);
			ImGui::SetNextItemWidth(120.0f);
			ImGui::DragFloat("v", &plan_values[i], 0.01f, -1e6f, 1e6f, "%.3f");
			ImGui::PopID();
			ImGui::SameLine();
			if ((i + 1) % 3 == 0) ImGui::NewLine();
		}
	}

	// ---- Range mode: start, end, step -> Generate plan ----
	if (plan_mode == 1) {
		static float rStart = 1.0f, rEnd = 1.8f, rStep = 0.1f;

		ImGui::SetNextItemWidth(120.0f);
		ImGui::InputFloat("Start", &rStart, 0.0f, 0.0f, "%.6f");
		ImGui::SetNextItemWidth(120.0f);
		ImGui::InputFloat("End", &rEnd, 0.0f, 0.0f, "%.6f");
		ImGui::SetNextItemWidth(120.0f);
		ImGui::InputFloat("Step", &rStep, 0.0f, 0.0f, "%.6f");

		// Helper that tolerates floating error and decreasing ranges
		auto buildRange = [](float a, float b, float s, int maxN) {
			std::vector<float> out;
			if (s == 0.0f || maxN <= 0) return out;

			// Correct step sign to move from a toward b
			if ((b > a && s < 0.0f) || (b < a && s > 0.0f)) s = -s;

			const float eps = 1e-6f;
			float x = a;
			int   n = 0;
			if (s > 0.0f) {
				while (x <= b + eps && n < maxN) { out.push_back(x); x += s; ++n; }
			}
			else {
				while (x >= b - eps && n < maxN) { out.push_back(x); x += s; ++n; }
			}
			return out;
			};

		if (ImGui::Button("Generate plan")) {
			auto v = buildRange(rStart, rEnd, rStep, kMaxPlanValues);
			if (!v.empty()) {
				plan_values = std::move(v);
				results_values.resize(plan_values.size(), 0);
			}
		}
		ImGui::SameLine();
		ImGui::Text("%d values planned", (int)plan_values.size());

		ImGui::Separator();
		// Optional preview / fine touch-up
		for (int i = 0; i < (int)plan_values.size(); ++i) {
			ImGui::PushID(i);
			ImGui::SetNextItemWidth(120.0f);
			ImGui::DragFloat("v", &plan_values[i], 0.01f, -1e6f, 1e6f, "%.3f");
			ImGui::PopID();
			ImGui::SameLine();
			if ((i + 1) % 3 == 0) ImGui::NewLine();
		}
	}

	ImGui::Separator();

	int type_int = static_cast<int>(exp_type);
	ImGui::Text("Mode:");
	ImGui::RadioButton("Viscosity", &type_int, (int)ExperimentType::VISCOSITY); ImGui::SameLine();
	ImGui::RadioButton("Flowrate", &type_int, (int)ExperimentType::FLOWRATE);    ImGui::SameLine();
	ImGui::RadioButton("Layer", &type_int, (int)ExperimentType::LAYER);    ImGui::SameLine();
	ImGui::RadioButton("Speed", &type_int, (int)ExperimentType::SPEED);          ImGui::SameLine();
	ImGui::RadioButton("Linked S&F", &type_int, (int)ExperimentType::LINKED_SPEED_FLOW);
	exp_type = static_cast<ExperimentType>(type_int);

	ImGui::Separator();


	ImGui::Checkbox("Save results", &store_results);
	ImGui::InputInt("Repeats per value", &exp_cfg.repeats);
	exp_cfg.repeats = std::max(1, exp_cfg.repeats);

	ImGui::Checkbox("Auto-start next run", &exp_cfg.auto_start);

	ImGui::Separator();
	ImGui::Text("Fixed parameters (used for modes where they are not varied):");
	ImGui::DragFloat("Fixed viscosity A", &exp_cfg.fixed_viscosity_a, 0.5f, 0.0f, 10000.0f, "%.3f");
	ImGui::DragFloat("Viscosity B ratio", &exp_cfg.viscosity_b_ratio, 0.01f, 0.0f, 10.0f, "%.3f");
	ImGui::DragFloat("Fixed flow override", &exp_cfg.fixed_flow_override, 0.01f, 0.0f, 5.0f, "%.3f");
	ImGui::DragFloat("Fixed print speed (F)", &exp_cfg.fixed_print_speed, 1.0f, 1.0f, 1e2f, "%.1f");
	ImGui::DragFloat("Fixed layer_height", &exp_cfg.fixed_layer_height, 0.05f, 0.0f, 2.0f, "%.1f");
	ImGui::DragFloat("Fixed travel speed (F)", &exp_cfg.fixed_travel_speed, 1.0f, 1.0f, 1e2f, "%.1f");


	if (exp_type == ExperimentType::LINKED_SPEED_FLOW) {
		ImGui::Text("Linked tuning (speed & flow are functions of plan v):");
		ImGui::DragFloat("Speed gain", &exp_cfg.link_speed_gain, 0.1f, -1e6f, 1e6f, "%.3f");
		ImGui::DragFloat("Speed bias", &exp_cfg.link_speed_bias, 1.0f, -1e6f, 1e6f, "%.3f");
		ImGui::DragFloat("Flow gain", &exp_cfg.link_flow_gain, 0.1f, -1e6f, 1e6f, "%.3f");
		ImGui::DragFloat("Flow bias", &exp_cfg.link_flow_bias, 0.1f, -1e6f, 1e6f, "%.3f");
	}


	ImGui::Separator();
	ImGui::Text("Current feedrates used by GCode:");
	ImGui::Text("  travel F=%.1f   print F=%.1f", travel_speed, print_speed);

	if (!exp_running) {
		if (ImGui::Button("Start batch")) {
			startExperiment(exp_type);
		}
	}
	else {
		if (ImGui::Button("Stop batch")) {
			stopExperiment();
		}
		ImGui::SameLine();
		if (!sim.isRunning() && ImGui::Button("Resume this run")) {
			sim.start();
		}
	}

	// simple progress
	if (total_runs_planned > 0) {
		float frac = float(total_runs_done) / float(total_runs_planned);
		ImGui::ProgressBar(frac, ImVec2(-FLT_MIN, 0), (std::to_string(total_runs_done) + "/" + std::to_string(total_runs_planned)).c_str());
	}

	ImGui::Text("Run index: %d / %d", exp_index + 1, (int)plan_values.size());
	ImGui::Text("Repeat:    %d / %d", exp_repeat + 1, exp_cfg.repeats);
	ImGui::Text("Tag: %s", currentRunTag().c_str());

	ImGui::End();

	// ---------------- Simulation controls (kept) ----------------
	ImGui::Begin("Simulation");

	if (sim.isRunning()) {
		if (ImGui::SmallButton("Stop simulation")) {
			sim.stop();
		}
	}
	else {
		if (ImGui::ArrowButton("Pause simulation", 1)) {
			sim.start();
		}
	}

	if (ImGui::SmallButton("Reset simulation")) {
		sim.reset();
		controller.reset();
		createGCode();
	}

	if (ImGui::Button("Save current state")) {
		saveOutput(true);
	}

	if (ImGui::Button("Compute Stats")) {
		saveOutput(false);
	}


	ImGui::Text("time %.5f s", sim.getTime());
	ImGui::DragFloat("Flow override", &settings.flow_override, 1.0, 0.0f, 100.0f);

	if (ImGui::TreeNode("Graphics"))
	{
		static bool transparency = true;
		if (ImGui::Checkbox("Particle transparency", &transparency)) {
			if (transparency) ps->setDisplayMode(ParticleSystemDisplayMode::POINT_SPRITE_SHADED);
			else ps->setDisplayMode(ParticleSystemDisplayMode::POINT_SPRITE);
		}

		ImGui::Checkbox("Show 2D plot", &use_2Dplot);

		if (ImGui::Checkbox("Show Isosurface", &use_isosurface)) {
			if (use_isosurface) isosurface->show();
			else isosurface->hide();
		}

		static bool Nozzlestate = true;
		if (ImGui::Checkbox("Show Nozzle", &Nozzlestate)) {
			if (Nozzlestate) nozzle->show();
			else nozzle->hide();
		}

		static bool Pstate = true;
		if (ImGui::Checkbox("Show Particles", &Pstate)) {
			if (Pstate) ps->show();
			else ps->hide();
		}

		static bool Bstate = false;
		if (ImGui::Checkbox("Show Bins", &Bstate)) {
			if (Bstate) bs->show();
			else bs->hide();
		}

		static bool BBstate = false;
		if (ImGui::Checkbox("Show Boundaries", &BBstate)) {
			particle_shader->use();
			particle_shader->setInt("showBoundary", BBstate);
		}

		ImGui::TreePop();
	}

	if (ImGui::TreeNode("Physics")) {

		ImGui::Checkbox("Use emitter", &settings.use_emitter);
		ImGui::Checkbox("Use emitter init", &settings.use_emitter_init);

		ImGui::DragInt("Solver substep", &settings.solver_substep, 1, 1, 200);
		ImGui::DragInt("Pressure Solver iteration", &settings.pressure_iteration, 1, 1, 50);
		ImGui::DragInt("Visco Solver iteration", &settings.visco_iteration, 1, 1, 50);

		ImGui::InputFloat("Time step", &settings.max_timestep, 0.0, 0.02f);

		if (ImGui::InputFloat("Fluid particle mass", &settings.particleMass.value(), 0.1, 2.0)) {
			solver->use();
			settings.particleMass.sync(*solver);
		}
		if (ImGui::InputFloat("Rest density", &settings.restDensity.value(), 0.0, 2.0)) {
			solver->use();
			settings.restDensity.sync(*solver);
			particle_shader->use();
			settings.restDensity.sync(*particle_shader);
		}

		static float artificialPressureMultiplier = settings.artificialPressureMultiplier.value() * 1000.0f;
		if (ImGui::InputFloat("Pressure multiplier", &artificialPressureMultiplier, 0.0f, 10.0f)) {
			settings.artificialPressureMultiplier.value() = artificialPressureMultiplier * 0.001f;
			solver->use();
			settings.artificialPressureMultiplier.sync(*solver);
		}

		float artificialViscosityMultiplier = settings.artificialViscosityMultiplier.value() * 100.0f;
		if (ImGui::InputFloat("XPSH Viscosity", &artificialViscosityMultiplier, 0.0f, 300.0f)) {
			settings.artificialViscosityMultiplier = artificialViscosityMultiplier * 0.01f;
			solver->use();
			settings.artificialViscosityMultiplier.sync(*solver);
		}

		if (ImGui::InputFloat("Viscosity A", &settings.viscosity_a.value())) {
			solver->use();
			solver->setFloat("u_viscosity_a", settings.viscosity_a.value());
		}

		if (ImGui::InputFloat("Viscosity B", &settings.viscosity_b.value())) {
			solver->use();
			solver->setFloat("u_viscosity_b", settings.viscosity_b.value());
		}

		ImGui::TreePop();
	}

	if (ImGui::TreeNode("Vizualisation"))
	{
		static int colorMode = 0;
		static const char* options[] = { "Solid color", "Bin index", "Density", "Temperature", "Velocity", "Mass", "Neighbors" };
		if (ImGui::ListBox("Colored field", &colorMode, options, 7)) {
			particle_shader->use();
			particle_shader->setInt("colorCycle", colorMode);
			bin_shader->use();
			bin_shader->setInt("colorCycle", colorMode);
			texPlot->use();
			texPlot->setInt("colorMode", colorMode);
			isosurface_shader->use();
			isosurface_shader->setInt("colorMode", colorMode);
		}

		static int particleTest = 50;
		static int binTest = 1459;
		if (colorMode == 6) {
			if (ImGui::DragInt("Particle to test", &particleTest)) {
				particle_shader->use();
				particle_shader->setUInt("particleTest", particleTest);
				bin_shader->use();
				bin_shader->setUInt("particleTest", particleTest);
			}

			ImGui::LabelText("Bin selector", "");

			bool changed = false;

			ImGui::LabelText("Current Bin", std::to_string(binTest).c_str());

			if (ImGui::SmallButton("X+")) { binTest++; changed = true; }
			if (ImGui::SmallButton("X-")) { binTest--; changed = true; }
			if (ImGui::SmallButton("Y+")) { binTest += (settings.bb.x / settings.bWidth); changed = true; }
			if (ImGui::SmallButton("Y-")) { binTest -= (settings.bb.x / settings.bWidth); changed = true; }
			if (ImGui::SmallButton("Z+")) { binTest += int(settings.bb.y / settings.bWidth) * int(settings.bb.x / settings.bWidth); changed = true; }
			if (ImGui::SmallButton("Z-")) { binTest -= int(settings.bb.y / settings.bWidth) * int(settings.bb.x / settings.bWidth); changed = true; }

			if (changed) {
				bin_shader->use();
				bin_shader->setUInt("binTest", binTest);
			}
		}
		ImGui::TreePop();
	}

	ImGui::End();

	// ---------------- ColorMap (kept) ----------------
	ImGui::Begin("ColorMap");
	static int colorMap = 4;
	static const char* optionsMap[] = { "inferno", "blackbody", "plasma", "viridis", "warmcool", "parula", "jet" };
	if (ImGui::ListBox("ColorMap field", &colorMap, optionsMap, 7)) {
		texPlot->use();
		texPlot->setInt("colorMapSelection", colorMap);
		isosurface_shader->use();
		isosurface_shader->setInt("colorMapSelection", colorMap);
	}
	ImGui::End();

	// ---------------- 2D Plots (kept) ----------------
	if (use_2Dplot) {
		static bool first_frame = true;
		static int debug_layer = 0;
		float scale = 0.6;

		ImGui::Begin("Debug XY");
		ImGui::Image((void*)(intptr_t)texture_debugXY->id(), ImVec2(settings.tex_size.x * scale, settings.tex_size.y * scale), ImVec2(1, 1), ImVec2(0, 0));
		ImGui::End();

		ImGui::Begin("Debug XZ");
		ImGui::Image((void*)(intptr_t)texture_debugXZ->id(), ImVec2(settings.tex_size.x * scale, settings.tex_size.z * scale), ImVec2(1, 1), ImVec2(0, 0));
		ImGui::End();

		ImGui::Begin("Debug YZ");
		ImGui::SliderFloat("Layer", &offsetPlane, -40, 40);
		if (ImGui::Button("Save picture")) {
			std::string savePath = Dialog::saveFileDialog(Dialog::FileType::IMAGE);
			if (!savePath.size() == 0) {
				texture_debugXY->exportPNG(savePath + "XY");
				texture_debugXZ->exportPNG(savePath + "XZ");
				texture_debugYZ->exportPNG(savePath + "YZ");
			}
		}
		ImGui::Image((void*)(intptr_t)texture_debugYZ->id(), ImVec2(settings.tex_size.y * scale, settings.tex_size.z * scale), ImVec2(1, 1), ImVec2(0, 0));
		ImGui::End();
	}



	if (use_profile_plot) {
		ImGui::Begin("Profile");
		ImGui::Image((void*)(intptr_t)profile_texture->id(), ImVec2(512, 512), ImVec2(1, 1), ImVec2(0, 0));
		ImGui::End();
	}



	// --------------- Scene graph (kept) ---------------
	static int selected_sample_index = -1;
	static int node_clicked = -1;
	static shared<RenderableObject> selected_renderable = nullptr;  // Track selected 3D scene object

	std::function<void(const std::list<shared<RenderableObject>>&)> traverseNodes = [&](const std::list<shared<RenderableObject>>& nodes) {
		for (auto& node : nodes) {
			ImGuiTreeNodeFlags node_flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick | ImGuiTreeNodeFlags_SpanAvailWidth;
			if (!node) continue;
			if (selected_renderable == node) {
				node_flags |= ImGuiTreeNodeFlags_Selected;
			}

			bool node_open = ImGui::TreeNodeEx(node->name().c_str(), node_flags);

			if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) {
				selected_renderable = node;
			}

			if (node_open) {
				traverseNodes(node->children());
				ImGui::TreePop();
			}
		}
		};

	ImGui::Begin("3D Scene Graph");
	traverseNodes(scene->nodes());
	ImGui::End();

	ImGui::Begin("Properties");
	if (selected_renderable) {
		selected_renderable->onRenderMenu();
	}
	ImGui::End();
}

void MainLayer::createShaders() {

	solver = sim.getSolver();

	particle_shader = Shader::create("particle", "assets/shaders/particle.vert", "assets/shaders/particle.frag", "", false);
	settings.setConstants(*particle_shader);
	particle_shader->compile();
	particle_shader->use();
	particle_shader->setVec3("lightPos", glm::vec3(0, -200, 1000));
	particle_shader->setInt("colorCycle", 4);

	bin_shader = Shader::create("bins", "assets/shaders/bin.vert", "assets/shaders/bin.frag", "", false);
	settings.setConstants(*bin_shader);
	bin_shader->compile();
	bin_shader->use();
	bin_shader->setInt("colorCycle", 4);

	renderer.addShader(particle_shader);
	renderer.addShader(bin_shader);

	isoGen = ComputeShader::create("isoGen", "assets/shaders/solver/isoGen.comp", false);
	settings.setConstants(*isoGen);
	isoGen->compile();
	isoGen->setWorkgroupLayout(settings.iWkgCount);

	texPlot = ComputeShader::create("texPlot", "assets/shaders/solver/texturePlot.comp", false);
	settings.setConstants(*texPlot);
	texPlot->compile();

	isosurface_shader = Shader::create("isosurface", "assets/shaders/isosurface.vert", "assets/shaders/isosurface.frag", "", false);
	isosurface_shader->supportShadows(false);
	isosurface_shader->supportMaterial(false);
	isosurface_shader->supportLights(false);
	isosurface_shader->supportEnvironment(false);
	isosurface_shader->supportTexture(false);
	settings.setConstants(*isosurface_shader);
	isosurface_shader->compile();


	profile_shader = Shader::create("profile", "assets/shaders/profile.vert", "assets/shaders/profile.frag", "");

}

void MainLayer::syncUniform() {
	if (need_sync) {
		particle_shader->use();
		settings.numParticles.sync(*particle_shader);
		settings.restDensity.sync(*particle_shader);

		isosurface_shader->use();
		settings.numParticles.sync(*isosurface_shader);
		settings.particleMass.sync(*isosurface_shader);
	}
}

std::string MainLayer::slice(Mesh_Ptr mesh, glm::vec3 normal, float h) {
	auto flat = Slicer::intersectMeshPlane(mesh, normal, h);
	auto polylines = Slicer::deflatten(flat);
	/*
	Vertices vertices;
	for (const auto& polyline : polylines) {
		for (const auto& line : polyline) {
			Vertex v;
			v.position = line;
			vertices.push_back(v);
		}
	}


	profile = Mesh::create("profile", vertices);
	profile->computeBoundingBox();
	profile_size = profile->getBoundingBox().size;
	profile->setDrawMode(GL_LINE_STRIP);*/
	
	return Slicer::toSVG(polylines, normal, 0.1f);
}

void MainLayer::plotIsoSurface() {
	if (use_isosurface) {
		if (isosurface->isHidden()) isosurface->show();
		isoGen->bindBuffer();
		volume->bindImage(0);
		isoGen->use();

		if (need_sync) {
			settings.numParticles.sync(*isoGen);
			settings.particleMass.sync(*isoGen);
		}

		isoGen->dispatch();
		isoGen->barrier(GL_ALL_BARRIER_BITS);
		isosurface->setIsoLevel(0.5);
		isosurface->compute();
	}
	else if (!isosurface->isHidden()) {
		isosurface->hide();
	}
}

void MainLayer::plotCutView() {
	if (use_2Dplot) {
		texPlot->bindBuffer();
		texPlot->use();

		settings.numParticles.sync(*texPlot);
		settings.particleMass.sync(*texPlot);

		plotXY();
		plotXZ();
		plotYZ();
	}
}


void MainLayer::plotXY() {
	int x = (settings.tex_size.x - 1) / settings.texWkgSize.x;
	int y = (settings.tex_size.y - 1) / settings.texWkgSize.y;

	texture_debugXY->bindImage(0);
	texPlot->setFloat("offsetPlane", offsetPlane);
	texPlot->setVec3("offset", glm::vec3(0, 0, 0));
	texPlot->setVec3("axis", glm::vec3(1, 1, 0));
	texPlot->dispatch(x, y);
	texPlot->barrier(GL_ALL_BARRIER_BITS);
}

void MainLayer::plotXZ() {
	int x = (settings.tex_size.x - 1) / settings.texWkgSize.x;
	int z = (settings.tex_size.z - 1) / settings.texWkgSize.z;

	texture_debugXZ->bindImage(0);
	texPlot->setFloat("offsetPlane", offsetPlane);
	texPlot->setVec3("axis", glm::vec3(1, 0, 1));
	texPlot->setVec3("offset", glm::vec3(0, 0, 0));
	texPlot->dispatch(x, z);
	texPlot->barrier(GL_ALL_BARRIER_BITS);
}

void MainLayer::plotYZ() {
	int y = (settings.tex_size.y - 1) / settings.texWkgSize.y;
	int z = (settings.tex_size.z - 1) / settings.texWkgSize.z;

	texture_debugYZ->bindImage(0);
	texPlot->setFloat("offsetPlane", offsetPlane);
	texPlot->setVec3("axis", glm::vec3(0, 1, 1));
	texPlot->setVec3("offset", glm::vec3(0, 0, 0));
	texPlot->dispatch(y, z);
	texPlot->barrier(GL_ALL_BARRIER_BITS);
}

void MainLayer::plotdepthMap() {
	int y = (settings.tex_size.y - 1) / settings.texWkgSize.y;
	int z = (settings.tex_size.z - 1) / settings.texWkgSize.z;

	texture_debug_depth->bindImage(0);

	depthPlot->setFloat("offsetPlane", offsetPlane);
	depthPlot->setVec3("axis", glm::vec3(0, 1, 1));
	depthPlot->setVec3("offset", glm::vec3(0, 0, 0));
	depthPlot->dispatch(y, z);
	depthPlot->barrier(GL_ALL_BARRIER_BITS);
}

void MainLayer::startExperiment(ExperimentType type) {
	store_results = true;
	exp_type = type;
	exp_index = 0;
	exp_repeat = 0;
	total_runs_done = 0;
	total_runs_planned = (int)plan_values.size() * exp_cfg.repeats;
	exp_running = (exp_type != ExperimentType::NONE) && (total_runs_planned > 0);

	// apply fixed params for non-varied ones (initial defaults)
	settings.viscosity_a = (exp_type == ExperimentType::VISCOSITY) ? settings.viscosity_a() : exp_cfg.fixed_viscosity_a;
	settings.viscosity_b = settings.viscosity_a.value() * exp_cfg.viscosity_b_ratio;


	settings.flow_override = (exp_type == ExperimentType::FLOWRATE || exp_type == ExperimentType::LINKED_SPEED_FLOW)
		? settings.flow_override : exp_cfg.fixed_flow_override;

	layer_height = (exp_type == ExperimentType::LAYER)
		? layer_height : exp_cfg.fixed_layer_height*10.0;

	print_speed = (exp_type == ExperimentType::SPEED || exp_type == ExperimentType::LINKED_SPEED_FLOW)
		? print_speed : exp_cfg.fixed_print_speed;
	travel_speed = exp_cfg.fixed_travel_speed;

	if (exp_running) {
		bool ok = prepareRunForCurrentPlan();
		if (ok && exp_cfg.auto_start) sim.start();
		if (!ok) stopExperiment();
	}
}

void MainLayer::stopExperiment() {
	exp_running = false;

	Console::printBoldSeparator();
	Console::success() << "All plan done." << Console::endl;
	Console::info() << "************************ Results ************************" << Console::endl;
	Console::info() << " value | width " << Console::endl;

	if (results_values.size() != plan_values.size()) {
		Console::error() << "Results count does not match plan count!" << Console::endl;
		Console::printBoldSeparator();
		return;
	}

	for (int i = 0; i < results_values.size(); i++) {
		Console::info() << " " << plan_values[i] << " | " << results_values[i] << " " << Console::endl;


	}
	Console::printBoldSeparator();
}

bool MainLayer::prepareRunForCurrentPlan() {
	if (!exp_running) return false;
	if (exp_index < 0 || exp_index >= (int)plan_values.size()) return false;

	float v = plan_values[exp_index];

	// Apply params per mode
	switch (exp_type) {
	case ExperimentType::VISCOSITY:
		/*
		if (v < 100) {
			settings.viscosity_a = v;
			settings.viscosity_b = v * exp_cfg.viscosity_b_ratio;
			settings.visco_iteration = 20; // unchanged

		}
		else {
			settings.viscosity_a = 100;
			settings.viscosity_b = 100 * exp_cfg.viscosity_b_ratio;
			settings.visco_iteration = 20;
		}*/


		ViscParams visco_a = paramsForViscosity(v);
		ViscParams visco_b = paramsForViscosity(v * exp_cfg.viscosity_b_ratio);

		settings.viscosity_a = visco_a.kV;
		settings.viscosity_b = visco_b.kV;
		settings.visco_iteration = visco_a.iterations;
		settings.artificialPressureMultiplier = visco_a.kP * 0.001;


		break;
	case ExperimentType::FLOWRATE:
		settings.flow_override = v;
		break;
	case ExperimentType::SPEED:
		print_speed = std::max(1.0f, v);
		travel_speed = exp_cfg.fixed_travel_speed; // keep travel fixed unless you want to vary both
		break;
	case ExperimentType::LAYER:
		//print_speed = std::max(1.0f, v);
		//travel_speed = exp_cfg.fixed_travel_speed; // keep travel fixed unless you want to vary both
		layer_height = v*10.0;
		break;
	case ExperimentType::LINKED_SPEED_FLOW:
		travel_speed = print_speed = std::max(1.0f, exp_cfg.link_speed_bias + exp_cfg.link_speed_gain * v);
		settings.flow_override = std::max(0.0f, exp_cfg.link_flow_bias + exp_cfg.link_flow_gain * v);
		break;
	default:
		break;
	}

	// push uniforms that changed
	solver->use();
	solver->setFloat("u_viscosity_a", settings.viscosity_a.value());
	solver->setFloat("u_viscosity_b", settings.viscosity_b.value());
	need_sync = true;

	// Rebuild GCode with new feedrates
	createGCode();
	controller.reset();
	sim.reset();

	return true;
}

void MainLayer::advanceAfterRun() {
	// one run finished
	total_runs_done++;

	// increment repeat
	exp_repeat++;
	if (exp_repeat >= exp_cfg.repeats) {
		exp_repeat = 0;
		exp_index++;
	}

	// all done?
	if (exp_index >= (int)plan_values.size()) {
		stopExperiment();
		return;
	}

	// prepare next
	if (prepareRunForCurrentPlan() && exp_cfg.auto_start) {
		sim.start();
	}
}

void MainLayer::saveRunOutputs(Mesh_Ptr mesh, const std::string& svg) {
	// Directory: ./out/<mode>/val_<v>/rep_<n>/
	const std::string mode = expTypeToString(exp_type);
	const float param_v = plan_values.empty() ? 0.0f : plan_values[std::min(exp_index, (int)plan_values.size() - 1)];
	const std::string val_str = sanitizeFloat(param_v);
	const std::string rep_str = std::to_string(exp_repeat + 1);

	fs::path baseDir = fs::path("./out") / mode;
	fs::path stlDir = baseDir / "stl";
	fs::path svgDir = baseDir / "svg";
	fs::path logDir = baseDir / "meta";


	std::error_code ec;
	fs::create_directories(baseDir, ec);
	fs::create_directories(stlDir, ec);
	fs::create_directories(svgDir, ec);
	fs::create_directories(logDir, ec);

	// Files
	const std::string tag = currentRunTag();
	const fs::path stlPath = stlDir / ("mesh_" + tag + ".stl");
	const fs::path svgPath = svgDir / ("slice_" + tag + ".svg");
	const fs::path logPath = logDir / ("runinfo_" + tag + ".txt");

	// Save mesh
	ModelLoader::saveMesh(mesh, stlPath.string());

	// Save SVG slice
	{
		std::ofstream ofs(svgPath.string(), std::ios::out | std::ios::trunc);
		if (ofs) ofs << svg;
	}

	// Save a small txt with parameters used (helps later)
	{
		std::ofstream ofs(logPath.string(), std::ios::out | std::ios::trunc);
		if (ofs) {
			ofs << "mode=" << mode << "\n";
			ofs << "value=" << param_v << "\n";
			ofs << "repeat=" << (exp_repeat + 1) << "/" << exp_cfg.repeats << "\n";
			ofs << "viscosity_a=" << settings.viscosity_a.value() << "\n";
			ofs << "viscosity_b=" << settings.viscosity_b.value() << "\n";
			ofs << "flow_override=" << settings.flow_override << "\n";
			ofs << "print_speed_F=" << print_speed << "\n";
			ofs << "layer_height=" << layer_height << "\n";
			ofs << "travel_speed_F=" << travel_speed << "\n";
		}
	}

	Console::info() << "Saved run: " << stlPath.string() << Console::endl;
}

// ==================== Utility helpers (naming, stringify) ====================
std::string MainLayer::sanitizeFloat(float v, int precision) {
	std::ostringstream ss;
	ss << std::fixed << std::setprecision(precision) << v;
	std::string s = ss.str();
	// replace locale decimal separator '.' with 'p' for safe filenames
	std::replace(s.begin(), s.end(), '.', 'p');
	return s;
}

std::string MainLayer::expTypeToString(ExperimentType t) {
	switch (t) {
	case ExperimentType::VISCOSITY:        return "viscosity";
	case ExperimentType::FLOWRATE:         return "flowrate";
	case ExperimentType::SPEED:            return "speed";
	case ExperimentType::LAYER:            return "layer";
	case ExperimentType::LINKED_SPEED_FLOW:return "linked";
	default:                               return "none";
	}
}

void MainLayer::saveOutput(bool store) {
	if (use_2Dplot && store) {
		float now = sim.getTime();
		std::string savePath = "./time" + std::to_string(plan_values[exp_index]);
		texture_debugXY->exportPNG(savePath + "XY.png");
		texture_debugXZ->exportPNG(savePath + "XZ.png");
		texture_debugYZ->exportPNG(savePath + "YZ.png");
	}

	if (use_isosurface) {
		Mesh_Ptr mesh = isosurface->saveMesh();
		//mesh->setShader("default");
		//mesh->setMaterial("cyan plastic");

		float avg_width = 0;
		float avg_height = 0;
		float iteration = 5;

		for (int i = 0; i < iteration; i++) {
			float min_x = 1e10f, max_x = -1e10f;
			float min_y = 1e10f, max_y = -1e10f;
			float offset = (i - iteration / 2);
			for (const auto& v : mesh->getVertices()) {
				if (v.position.y > -1 + offset && v.position.y < 1 + offset) {
					if (v.position.x < min_x) min_x = v.position.x;
					if (v.position.x > max_x) max_x = v.position.x;

					if (v.position.z < min_y) min_y = v.position.z;
					if (v.position.z > max_y) max_y = v.position.z;
				}
			}

			avg_width += (max_x - min_x) / 10.0;
			avg_height += (max_y - min_y) / 10.0;
		}

		avg_width /= iteration;
		avg_height /= iteration;

		MeshStats stats = mesh->computeMeshStats(0.1, 1.24);

		Console::printSeparator();
		Console::print() << "---------- Results ----------" << Console::endl;
		Console::print() << "| -> " << currentRunTag() << "|" << Console::endl;
		Console::print() << "Average width: " << avg_width << " mm" << Console::endl;
		Console::print() << "Average height: " << avg_height << " mm" << Console::endl;
		Console::print() << "Mesh Volume: " << stats.volume_mm3 << " mm3" << Console::endl;
		Console::print() << "Mesh Surface area: " << stats.surfaceArea_mm2 << " mm2" << Console::endl;
		Console::print() << "Mesh vertices: " << stats.numVertices << Console::endl;
		Console::print() << "Mesh triangles: " << stats.numTriangles << Console::endl;
		Console::print() << "Mesh mass: " << stats.mass_g << "g" << Console::endl;
		Console::print() << "Time : " << sim.getTime() << "s" << Console::endl;
		Console::printSeparator();



		#ifdef BEAD_TEST
			results_values[exp_index] = avg_width;
		#elif defined(COIL_TEST)
			results_values[exp_index] = stats.volume_mm3;
		#endif

		std::string svg = slice(mesh, glm::vec3(0, 1, 0), 0);

		if (store) saveRunOutputs(mesh, svg);
	}
}

std::string MainLayer::currentRunTag() const {
	std::ostringstream ss;
	float val = plan_values.empty() ? 0.0f : plan_values[std::min(exp_index, (int)plan_values.size() - 1)];
	ss << expTypeToString(exp_type)
		<< "_val-" << sanitizeFloat(val)
		<< "_rep-" << (exp_repeat + 1);
	return ss.str();
}

