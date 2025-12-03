#include "sim.h"
#include "../settings.h"
#include <GLFW/glfw3.h>

Sim::Sim(){}
Sim::~Sim() {}


void Sim::init() {
	Console::print() << "Setting up simulation" << Console::endl;

	//Compute Shaders
	solver = StagedComputeShader::create("solver", "assets/shaders/solver/solver.comp", 10, false);
	settings.setConstants(*solver);
	solver->compile();

	prefixSum = StagedComputeShader::create("prefixSum", "assets/shaders/solver/prefix.sum.comp", 4, false);
	settings.setConstants(*prefixSum);
	prefixSum->compile();


	//create particle system
	ps = ParticleSystem::create("ParticleSystem", settings.pThread);
	ps->setDisplayMode(ParticleSystemDisplayMode::POINT_SPRITE_SHADED);

	shared<Mesh> binInstance = Primitives::createQuadCube(settings.bWidth, false);
	binInstance->rename("bin");
	bs = ParticleSystem::create("BinSystem", settings.bThread);
	bs->setDisplayMode(ParticleSystemDisplayMode::POINT_SPRITE_SHADED);
	bs->setMesh(binInstance);
	bs->enableWireFrameMode();

	solver->setWorkgroupLayout(settings.pWkgCount);
	prefixSum->setWorkgroupLayout(settings.bWkgCount);

	
	//static_emitter = Primitives::createCylinder(1.6, 3, 10);
	static_emitter = Primitives::createCylinder(2, 1, 10);
	//static_emitter = Primitives::createCylinder(1.6*3, 3, 10);
	static_emitter->translate(glm::vec3(0, 0, 1.1 * 3.5));
	static_emitter->rotate(glm::vec3(0, 90*DEG_TO_RAD, 0));

	nozzle_emitter = ModelLoader::loadMesh("./assets/models/nozzle_emitter.stl");
	nozzle_emitter->scale(5);
	nozzle_emitter->applyMeshTransform();


	obstacle = ModelLoader::loadMesh("./assets/common/models/bunny.stl");
	obstacle->scale(3);
	obstacle->applyMeshTransform();

	MemoryManager::instance().resetBindings();


	Console::info() << "Generating particles..." << Console::endl;

	float spacing = settings.particleSpacing;
	cpu_position = std::vector<glm::vec4>();
	cpu_sdf = std::vector<glm::vec4>();
	//auto cpu_velocity = std::vector<glm::vec4>();
	cpu_temp = std::vector<glm::vec2>();
	cpu_meta = std::vector<glm::uvec4>();
	cpu_emitterPosition = std::vector<glm::vec4>();

	settings.numEmitter = 0;
	settings.numParticles = 0;
	std::vector<glm::vec3> positions;



	/**/
	static_emitter->computeBoundingBox();
	static_emitter->voxelize(spacing);
	positions = Voxelizer::getVoxelposition(static_emitter->getVoxels(), static_emitter->getBoundingBox(), spacing);

	/**/
	for (int i = 0; i < positions.size(); i++) {
		//cpu_position.push_back(glm::vec4(positions[i], 0));
		//cpu_temp.push_back(298.15); //ambient
		//cpu_meta.push_back(glm::uvec4(FLUID, settings.numParticles(), settings.numParticles(), 0.0));

		cpu_emitterPosition.push_back(glm::vec4(positions[i], 0.0));
		settings.numEmitter()++;
	}
	/**/

	/*
	floor->computeBoundingBox();
	floor->voxelizeSurface(spacing, 0.5);
	positions = Voxelizer::getVoxelposition(floor->getVoxels(), floor->getBoundingBox(), spacing);

	for (int i = 0; i < positions.size(); i++) {
		cpu_position.push_back(glm::vec4(positions[i] - glm::vec3(0,0,settings.smoothingRadius*5.0), 0));
		cpu_temp.push_back(glm::vec2(275.15 + 20, 0)); //ambient
		cpu_meta.push_back(glm::uvec4(BOUNDARY, settings.numParticles(), settings.numParticles(), 0.0));
		settings.numParticles()++;
	}/**/


	std::vector<glm::vec4> sdf;
	spacing = settings.particleSpacing;
	#ifdef USE_COLLIDER
	//nozzle_emitter->scale(1.1);
	nozzle_emitter->setPosition(glm::vec3(0, 0, 0));
	nozzle_emitter->computeBoundingBox();
	nozzle_emitter->voxelize(spacing, true);
	positions = Voxelizer::getVoxelposition(nozzle_emitter->getVoxels(), nozzle_emitter->getBoundingBox(), spacing);
	sdf = nozzle_emitter->getVoxelsSDF();

	for (int i = 0; i < positions.size(); i++) {
		cpu_position.push_back(glm::vec4(positions[i], 0));
		cpu_sdf.push_back(sdf[i]);
		//cpu_velocity.push_back(nozzle_emitter->computeSDF(positions[i]));
		cpu_temp.push_back(glm::vec2(275.15 + 220, 0));
		cpu_meta.push_back(glm::uvec4(BOUNDARY + 1, settings.numParticles(), settings.numParticles(), 0.0));
		settings.numParticles()++;

	}
	#endif

	/**/

	/*
	obstacle->setPosition(glm::vec3(0, 0, 0));
	obstacle->computeBoundingBox();
	obstacle->voxelize(spacing, true);
	positions = Voxelizer::getVoxelposition(obstacle->getVoxels(), obstacle->getBoundingBox(), spacing);
	sdf = obstacle->getVoxelsSDF();

	for (int i = 0; i < positions.size(); i++) {
		cpu_position.push_back(glm::vec4(positions[i], 0));
		cpu_sdf.push_back(sdf[i]);
		cpu_temp.push_back(glm::vec2(275.15 + 220, 0));
		cpu_meta.push_back(glm::uvec4(BOUNDARY + 2, settings.numParticles(), settings.numParticles(), 0.0));
		settings.numParticles()++;

	}
	/**/
	
	/*
	Mesh_Ptr init_fluid = Primitives::createBox(2, 20, 80);
	init_fluid->setPosition(glm::vec3(0, 0, 40));
	init_fluid->voxelize(spacing * 0.8);
	positions = Voxelizer::getVoxelposition(init_fluid->getVoxels(), init_fluid->getBoundingBox(), spacing * 0.8);


	for (int i = 0; i < positions.size(); i++) {
		cpu_position.push_back(glm::vec4(positions[i], 0));
		cpu_sdf.push_back(glm::vec4(0));
		//cpu_velocity.push_back(glm::vec4(0));
		cpu_temp.push_back(glm::vec2(275.15 + 200, 0)); //ambient
		cpu_meta.push_back(glm::uvec4(FLUID_A, settings.numParticles(), settings.numParticles(), 0.0));
		settings.numParticles()++;
	}*/

	/*
	if (settings.use_emitter_init) { 
		Mesh_Ptr init_fluid = Primitives::createCylinder(4.5, 18, 10);
		init_fluid->setPosition(glm::vec3(0, 0, 23));
		init_fluid->rotate(glm::vec3(0, 90.0f * DEG_TO_RAD, 0));
		init_fluid->computeBoundingBox();
		init_fluid->voxelize(spacing * 0.8);
		positions = Voxelizer::getVoxelposition(init_fluid->getVoxels(), init_fluid->getBoundingBox(), spacing * 0.8);

		
		for (int i = 0; i < positions.size(); i++) {
			cpu_position.push_back(glm::vec4(positions[i], 0));
			cpu_sdf.push_back(glm::vec4(0));
			//cpu_velocity.push_back(glm::vec4(0));
			cpu_temp.push_back(glm::vec2(275.15 + 200, 0)); //ambient
			cpu_meta.push_back(glm::uvec4(FLUID_A, settings.numParticles(), settings.numParticles(), 0.0));
			settings.numParticles()++;
		}

		init_fluid = Primitives::createCone(13, 5, 10);
		init_fluid->setPosition(glm::vec3(0, 0, 23));
		init_fluid->rotate(glm::vec3(0.0f, 90.0f * DEG_TO_RAD, 0.0f));
		init_fluid->computeBoundingBox();
		init_fluid->voxelize(spacing * 0.8);
		positions = Voxelizer::getVoxelposition(init_fluid->getVoxels(), init_fluid->getBoundingBox(), spacing * 0.8);


		
		for (int i = 0; i < positions.size(); i++) {
			cpu_position.push_back(glm::vec4(positions[i], 0));
			cpu_sdf.push_back(glm::vec4(0));
			//cpu_velocity.push_back(glm::vec4(0));
			cpu_temp.push_back(glm::vec2(275.15 + 200, 0)); //ambient
			cpu_meta.push_back(glm::uvec4(FLUID_A, settings.numParticles(), settings.numParticles(), 0.0));
			settings.numParticles()++;
		}

	}
	*/
	settings.init_nummparticle = cpu_position.size();
	while (cpu_position.size() < settings.max_pThread) {
		cpu_position.push_back(glm::vec4(0.0, 0.0, 0.0, 0.0));
		cpu_sdf.push_back(glm::vec4(0));
		//cpu_velocity.push_back(glm::vec4(0));
		cpu_temp.push_back(glm::vec2(298.15, 0));
		cpu_meta.push_back(glm::uvec4(FLUID_EMITTER, settings.numParticles(), settings.numParticles(), settings.numParticles()));
	}
}

void Sim::reset(){
	Console::printSeparator();
	Console::print() << "Reset simulation" << Console::endl;

	//std::lock_guard<std::mutex> lock(sim_mutex);
	elapsedTime = 0;
	lastSpawTime = 0;

	auto& memory = MemoryManager::instance();
	memory.resetBindings();
	m_current_flowrate = 0;
	Console::info() << "Uploading buffer on device..." << Console::endl;

	settings.numParticles = settings.init_nummparticle;
	settings.pThread = settings.numParticles();
	settings.pWkgCount = (settings.pThread + settings.pWkgSize - 1) / settings.pWkgSize; //Total number of workgroup needed
	settings.blockSize = std::floor(log2f(settings.bThread));
	settings.blocks = (settings.bThread + settings.blockSize - 1) / settings.blockSize;
	settings.bWkgCount = (settings.blocks + settings.bWkgSize - 1) / settings.bWkgSize; //Total number of workgroup needed
	settings.emitter_transform = glm::mat4(1);
	//settings.emitter_transform.sync(*solver);
	emitterVolume = cpu_emitterPosition.size() * settings.particleVolume;


	solver->setWorkgroupLayout(settings.pWkgCount);
	prefixSum->setWorkgroupLayout(settings.bWkgCount);

	ps->setInstancesCount(settings.max_pThread);
	ps->setActiveInstancesCount(settings.pThread);
	bs->setInstancesCount(settings.bThread);

	syncUniform();

	Console::info() << "Uploading buffer on device..." << Console::endl;
	memory.writeBuffer("position_buffer", cpu_position);
	memory.writeBuffer("predicted_position_buffer", cpu_position);
	memory.writeBuffer("last_position_buffer", cpu_position);
	memory.writeBuffer("correction_buffer", cpu_sdf);
	//ps->writeField("velocity_buffer", cpu_velocity);
	memory.writeBuffer("temperature_buffer", cpu_temp);
	memory.writeBuffer("meta_buffer", cpu_meta);
	memory.writeBuffer("emitter_position_buffer", cpu_emitterPosition);

	memory.clearBuffer("velocity_buffer");
	memory.clearBuffer("density_buffer");
	memory.clearBuffer("lambda_buffer");
	//memory.clearBuffer("stress_buffer");

	memory.resetBindings();
}

void Sim::start(){
	running = true;
}

void Sim::stop(){
	running = false;
}

bool Sim::isRunning() const {
	return running;
}

void Sim::spawn(){
	settings.pThread = settings.numParticles() + settings.numEmitter();
	settings.pWkgCount = (settings.pThread + settings.pWkgSize - 1) / settings.pWkgSize; //Total number of workgroup needed
	solver->setWorkgroupLayout(settings.pWkgCount);
	ps->setActiveInstancesCount(settings.pThread);

	solver->use();
	settings.numEmitter.sync(*solver);
	//settings.numParticles.sync(*solver);

	solver->execute(8);

	settings.numParticles() += settings.numEmitter();
	settings.numParticles.sync(*solver);

	solver->use();
	solver->setUInt("u_numEmitter", 0);
	settings.numParticles.sync(*solver);

	settings.pThread = settings.numParticles();
	settings.pWkgCount = (settings.pThread + settings.pWkgSize - 1) / settings.pWkgSize; //Total number of workgroup needed
	solver->setWorkgroupLayout(settings.pWkgCount);
	ps->setActiveInstancesCount(settings.pThread);
	
}

void Sim::nns(){
	
	prefixSum->use();

	prefixSum->setUInt("dataSize", settings.bThread); //data size
	prefixSum->setUInt("blockSize", settings.blockSize); //block size
	prefixSum->execute(4);// clear bins

	solver->use();
	solver->execute(0); //Place particles in bins

	prefixSum->use();
	prefixSum->execute(0);// local prefix sum

	//Binary tree on rightmost element of blocks
	GLuint steps = settings.blockSize;
	UniformObject<GLuint> space("space");
	space.value() = 1;

	for (GLuint step = 0; step < steps; step++) {
		// Calls the parallel operation

		space.sync(*prefixSum);
		prefixSum->execute(1);
		prefixSum->execute(2);

		space.value() *= 2;
	}
	prefixSum->execute(3);

	solver->use();
	solver->execute(1); //Sort
	
}

float Sim::getTime(){
	return elapsedTime;
}


void Sim::control(float vx, float vy, float ve){
	//simulator.control(vx, vy, 0, ve);
	settings.flow_override = ve;
}

void Sim::step(Timestep ts) {

	if (ts.getSeconds() <= 0.0f) return;
	if (ts.getSeconds() > settings.max_timestep) ts = Timestep(settings.max_timestep);

	if (running) {

		solver->bindBuffer();
		prefixSum->bindBuffer();

		GPU_PROFILE(solver_total_time,

			for (int i = 0; i < settings.solver_substep; i++) {

			elapsedTime += ts / settings.solver_substep;

			solver->use();
			settings.setTimestep(ts.getSeconds());
			settings.dt.sync(*solver);

			settings.emitter_transform = glm::mat4(1);
			settings.emitter_transform = glm::translate(settings.emitter_transform(), glm::vec3(nozzle_position));
			settings.emitter_transform.sync(*solver);

			//obstacle->setPosition(getBunnyPosition());
			solver->setMat4("u_bunnyTransform", obstacle->transform());

			static float acc_volume = 0.0f;
			float flow_rate = m_current_flowrate;
			acc_volume += flow_rate * ts.getSeconds() / settings.solver_substep;

			if (flow_rate * ts.getSeconds() / settings.solver_substep > emitterVolume) {
				Console::warn() << "Flowrate too high for current emitter size! Increase emitter size or decrease flowrate" << Console::endl;
			}
			//Console::info() << "Flowrate: " << flow_rate << " mm3/s, Acc volume: " << acc_volume << " mm3" << Console::endl;


			if (settings.use_emitter && acc_volume < -emitterVolume) {
				solver->setInt("retract", 1);
			}else solver->setInt("retract", 0);

			if (settings.use_emitter && acc_volume > emitterVolume) {
				spawn();
				acc_volume -= emitterVolume;
			}



			
				solver->execute(2);


				GPU_PROFILE(nns_time,
					nns();
				)


				for (int j = 0; j < settings.pressure_iteration; j++) { // Pressure solve
					solver->execute(3);
					solver->execute(4);
					solver->execute(5);
				}
				for (int j = 0; j < settings.visco_iteration; j++) { // Viscosity solve
					solver->execute(6);
				}
				
			}
		)
		MemoryManager::instance().resetBindings();
	}
}


void Sim::syncUniform(){

	solver->use();

	//settings.timestep.sync(*solver);
	settings.particleMass.sync(*solver);
	settings.restDensity.sync(*solver);
	settings.numParticles.sync(*solver);
	settings.numEmitter.sync(*solver);
	settings.dt.sync(*solver);
	settings.viscosity_a.sync(*solver);
	settings.viscosity_b.sync(*solver);
	settings.artificialViscosityMultiplier.sync(*solver);
	settings.artificialPressureMultiplier.sync(*solver);

	settings.emitter_transform = glm::mat4(1);
	//settings.emitter_transform = glm::translate(settings.emitter_transform(), glm::vec3(-20, 0, 50));
	settings.emitter_transform.sync(*solver);

	prefixSum->use();
	prefixSum->setUInt("dataSize", settings.bThread); //data size
	prefixSum->setUInt("blockSize", settings.blockSize); //block size
}

void Sim::setNozzlePosition(const glm::vec3& pos){
	nozzle_position = pos;
}

void Sim::setFlowrate(float flow){
	m_current_flowrate = flow;
}

const glm::vec3& Sim::getNozzlePosition() const{
	return nozzle_position;
}

ParticleSystem_Ptr Sim::getParticles() const
{
	return ps;
}

ParticleSystem_Ptr Sim::getBins() const
{
	return bs;
}

glm::vec3 Sim::animationBunny(float t) const {

	if (t < 10) t = 0;
	t *= 10.1f;

	float x = sin(t) + 0.5f * cos(2.0f * t);
	float y = cos(3.0f * t) + 0.5f * sin(4.0f * t);
	float z = sin(5.0f * t) + 0.5f * cos(6.0f * t);
	return glm::vec3(x*0.8, y*0.5, z)*10.0f;
}

StagedComputeShader_Ptr Sim::getSolver()
{
	return solver;
}
