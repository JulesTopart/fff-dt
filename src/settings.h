#pragma once
#include <Merlin.h>
#include <glm/glm.hpp>


#define PROFILE(VAR, CODE) double start_ ## VAR ## _time = glfwGetTime(); CODE VAR = (glfwGetTime() - start_ ## VAR ## _time)*1000.0;
#define GPU_PROFILE(VAR, CODE) double start_ ## VAR ## _time = glfwGetTime(); CODE glFinish(); VAR = (glfwGetTime() - start_ ## VAR ## _time)*1000.0;

#define PROFILE_BEGIN(STARTVAR) STARTVAR = glfwGetTime();
#define PROFILE_END(STARTVAR, VAR) VAR = (glfwGetTime() - STARTVAR)*1000.0

#define BEAD_TEST
//#define MULTI_BEAD_TEST
//#define COIL_TEST
#define USE_COLLIDER

struct Bin {
	GLuint count; //particle count in the bin
	GLuint sum;   //global sum
	GLuint csum;  //local sum
	GLuint index; //bin index
};

struct CopyContent {
	alignas(16) glm::vec4 lx;
	alignas(16) glm::vec4 x;
	alignas(16) glm::vec4 p;
	alignas(16) glm::vec4 dp;
	alignas(16) glm::vec4 v;
	//alignas(64) glm::mat4 F;
	//alignas(64) glm::mat4 L;
	alignas(16) glm::uvec4 meta;
	alignas(16) glm::uvec4 temperature_density_lambda;
};


using namespace Merlin;

struct Settings {
	Settings() {};


	const float particleSpacing = 0.5;//1e-1 mm
	//const float particleVolume = (4.0 / 3.0) * glm::pi<float>() * particleRadius * particleRadius * particleRadius;
	
	const float smoothingRadius = 2.0 * particleSpacing;
	const float bWidth = smoothingRadius;
	const float volumeWidth = 0.4;

	//Solver settings
	int solver_substep = 5;//4
	int pressure_iteration = 3;//3
	int visco_iteration = 5;//20
	float overRelaxation = 1.0;

	//Boundary Volume dimensions
	#ifdef BEAD_TEST
	glm::vec3 bb = glm::vec3(50, 80, 30);
	#elif defined(MULTI_BEAD_TEST)
	glm::vec3 bb = glm::vec3(50, 50, 40);
	#elif defined(COIL_TEST)
	glm::vec3 bb = glm::vec3(40, 40, 50);
	#elif defined(LOGO)
	glm::vec3 bb = glm::vec3(300, 80, 30);
	#endif

	// Physics Parameters
	float timestep									= 0.001;
	float max_timestep								= 0.001;
	Uniform<float> dt								= Uniform<float>("u_dt", timestep / solver_substep);
	Uniform<float> restDensity						= Uniform<float>("u_rho0", 1.24);//mg/cm3
	//Uniform<float> particleMass 					= Uniform<float>("u_mass", 0.124);//mg
	Uniform<float> particleMass 					= Uniform<float>("u_mass", 0.09);//mg
	//Uniform<float> particleMass					= Uniform<float>("u_mass", 0.2);
	const float particleVolume						= (1.0/1.39)*1e-3*(particleMass()/restDensity());//mm/3
	Uniform<float> viscosity_a						= Uniform<float>("u_viscosity_a", 500);
	Uniform<float> viscosity_b						= Uniform<float>("u_viscosity_b", 500);
	//Uniform<float> artificialViscosityMultiplier	= Uniform<float>("u_artificialViscosityMultiplier", 47 * 0.001);
	Uniform<float> artificialViscosityMultiplier	= Uniform<float>("u_artificialViscosityMultiplier", 0.0 * 0.001);
	Uniform<float> artificialPressureMultiplier		= Uniform<float>("u_artificialPressureMultiplier",   0.5 * 0.001); //2.6 * 0.001

	//calculated (don't change value here)
	GLuint init_nummparticle = 0;
	Uniform<GLuint> numParticles					= Uniform<GLuint>("u_numParticles", 0);
	Uniform<GLuint> numEmitter						= Uniform<GLuint>("u_numEmitter", 0);

	Uniform<glm::mat4> emitter_transform			= Uniform<glm::mat4>("u_emitterTransform", glm::mat4(1));

	//Flow
	float flow_override = 2.513;
	bool use_emitter = true;
	bool use_emitter_init = false;

	//GPU Threading settings
	GLuint max_pThread = 1000000; //Max Number of particles (thread) (10 milion)
	GLuint pThread = 1;
	GLuint pWkgSize = 512; //Number of thread per workgroup
	GLuint pWkgCount = (pThread + pWkgSize - 1) / pWkgSize; //Total number of workgroup needed

	GLuint bWkgSize = 512; //Number of thread per workgroup
	GLuint bThread = int(bb.x / (bWidth)) * int(bb.y / (bWidth)) * int(bb.z / (bWidth)); //Total number of bin (thread)
	GLuint blockSize = floor(log2f(bThread));
	GLuint blocks = (bThread + blockSize - 1) / blockSize;
	GLuint bWkgCount = (blocks + bWkgSize - 1) / bWkgSize; //Total number of workgroup needed

	GLuint cThread = max_pThread * 32; //Max Number of constraint (thread) (32 * 1M)

	glm::ivec3 iWkgSize = glm::ivec3(8); //Number of thread per workgroup
	glm::ivec3 volume_size = glm::ivec3(int(bb.x / (volumeWidth)), int(bb.y / (volumeWidth)), int((bb.z) / (volumeWidth)));
	glm::ivec3 iWkgCount = (volume_size + iWkgSize - glm::ivec3(1)) / iWkgSize; //Total number of workgroup needed

	glm::ivec3 texWkgSize = glm::ivec3(8, 8, 1); //Number of thread per workgroup
	glm::ivec3 tex_size = glm::ivec3(int(bb.x / (volumeWidth * 0.1)), int(bb.y / (volumeWidth * 0.1)), int(bb.z / (volumeWidth * 0.1)));

	inline void setTimestep(float t) {
		timestep = t;
		dt = timestep / solver_substep;
	}

	inline void setConstants(ShaderBase& shader) {
		shader.setConstVec3("cst_domain", bb);
		shader.setConstVec3("cst_halfdomain", bb*glm::vec3(0.5));
		shader.setConstVec3("cst_boundaryMin", bb*glm::vec3(-0.5, -0.5, -0.02) + glm::vec3(0, 0, 0.0));
		shader.setConstVec3("cst_boundaryMax", bb*glm::vec3(0.5, 0.5, 0.98) + glm::vec3(0, 0, 0.0));

		shader.setConstFloat("cst_particleRadius", particleSpacing);
		shader.setConstFloat("cst_smoothingRadius", smoothingRadius);
		shader.setConstFloat("cst_binSize", bWidth);
		shader.setConstUVec3("cst_binMax", glm::uvec3(bb / bWidth));
		shader.setConstUInt("cst_binCount", bThread);

		shader.define("PTHREAD", std::to_string(pWkgSize));
		shader.define("BTHREAD", std::to_string(bWkgSize));
	}


	SINGLETON(Settings)
};

extern Settings settings;


/*********** Stats ***********/
extern double nns_time;
extern double jacobi_time;
extern double solver_substep_time;
extern double solver_total_time;
extern double render_time;
extern double render_start_time;
extern double total_time; 
extern double total_start_time;