//This file was automatically generated 
//DO NOT CHANGE !
#version 430 core

#include "../common/shaders/graphics/colors.comp"
#include "./common/uniforms.comp"
#include "./common/constants.comp"
#include "./common/buffers.comp"
#include "./common/nns.comp"
#include "./solver/sph.kernels.comp" 

in vec3 position;
in vec3 normal;
in vec3 color;

out vec4 FragColor;

uniform vec3 viewPos;
uniform int colorMapSelection = 0;
uniform int colorMode = 0;


float computeVonMisesStress(mat3 sigma) {

    float vonMises = sqrt(
        0.5 * (
            pow(sigma[0][0] - sigma[1][1], 2.0) + 
            pow(sigma[1][1] - sigma[2][2], 2.0) + 
            pow(sigma[2][2] - sigma[0][0], 2.0) + 
            6.0 * (pow(sigma[0][1], 2.0) + pow(sigma[1][2], 2.0) + pow(sigma[2][0], 2.0))
        )
    );

    return vonMises;
}



vec2 mapHeight(in vec3 position){
	float height = 0;
	float density = 0;
	float n = 0;
	OVERNNS
		float dist = distance(b_xj.xyz, position);
		if(dist < 1e-6 || dist > H*0.9) continue;
		float Wij = poly6(dist);

		if(is_solid(j) || is_fluid(j)){
			height += b_pj.z;
			density += mi * Wij;
			n++;
		}
	OVERNNS_END
	if(n > 1) height /= n;
	return vec2(height, density);
}

vec2 mapHeat(in vec3 position){
	float temp = 0;
	float density = 0;
	float n = 0;
	OVERNNS
		float dist = distance(b_xj.xyz, position);
		if(dist < 1e-6 || dist > H) continue;
		float Wij = poly6(dist);

		if(is_solid(j) || is_fluid(j)){
			temp += b_Tj;
			density += mi * Wij;
			n++;
		}
	OVERNNS_END
	if(n > 1) temp /= n;
	//if(position.z > 20) temp = 275.98 + 220;

	return vec2(temp, density);
}

float mapStress(in vec3 position){
	float vm_stress = 0;
	float n = 0;
	OVERNNS
		float dist = distance(b_xj.xyz, position);
		if(dist < 1e-6 || dist > H) continue;
		float Wij = poly6(dist);

		if(is_solid(j)){
			n++;

			mat3 sigmaJ = mat3(0);
			sigmaJ[0] = b_sigma_j[0].xyz; // First column
			sigmaJ[1] = b_sigma_j[1].xyz; // Second column
			sigmaJ[2] = b_sigma_j[2].xyz; // Third column

			vm_stress += computeVonMisesStress(sigmaJ);
		}
	OVERNNS_END
	if(n > 1) vm_stress /= n;

	return vm_stress;
}

float interpolatePhaseAB(in vec3 position){
	float est_phaseA = 0;
	float est_phaseB = 0;
	float est_phaseNone = 0;

	OVERNNS
		float dist = distance(b_xj.xyz, position);
		if(dist > H) continue;
		float Wij = poly6(dist);

		if(phaseAB(j) == 0)
			est_phaseA += mj * Wij;
		else if(phaseAB(j) == 1)
			est_phaseB += mj * Wij;
		else 
			est_phaseNone += mj * Wij;
	OVERNNS_END

	if(est_phaseA > est_phaseB) 
		return 0.0;
	else 
		return 1.0;
}
void main() {
	
    vec2 mappedV = mapHeat(position);//length(_normal);
    vec2 heightV = mapHeight(position);//length(_normal);
    float temp = mappedV.x;//length(_normal);
    float height = heightV.x;//length(_normal);
    float density = mappedV.y;//length(_normal);
    float stress = mapStress(position);//length(_normal);
	if(temp < 275.5 + 25.0) temp = 275.5 + 25.0;



	Palette palette = inferno;


	float minValue = 0;
	float maxValue = 0;
	float value = 0;

	if(colorMapSelection == 0)
		palette = inferno;
	else if(colorMapSelection == 1)
		palette = blackbody;
	else if(colorMapSelection == 2)
		palette = plasma;
	else if(colorMapSelection == 3)
		palette = viridis;
	else if(colorMapSelection == 4)
		palette = warmcool;
	else if(colorMapSelection == 5)
		palette = parula;
	else if(colorMapSelection == 6)
		palette = jet;
		

	if(colorMode == 0){ // solid color

		vec3 color = vec3(0.8, 0.8, 0.8);
		vec3 colorA = vec3(3, 144, 252)/255.0;
		vec3 colorB = vec3(0.0, 0.0, 0.8);

		float phaseAB = interpolatePhaseAB(position);

		if(phaseAB == 0){
			color = colorA;
		}else if(phaseAB == 1){
			color = colorB;
		}

		FragColor = vec4(color,1);
		
		vec3 viewDir = normalize(viewPos - position);
		vec3 lightDir = normalize(vec3(1,1,5));
		float diff = max(dot(normal, lightDir), 0.0);
		float spec = 0;
		if(diff != 0){
			vec3 halfwayDir = normalize(lightDir + viewDir);
			spec = pow(max(dot(normal, halfwayDir), 0.0), 0.1); //blinn
		}

		vec3 ambient = 0.6 * FragColor.rgb;
		vec3 diffuse = 0.4 * diff * FragColor.rgb;
		vec3 specular = 0.3 * spec * FragColor.rgb;
		//FragColor.rgb = N* 0.9 + FragColor.rgb * 0.1;
		//FragColor.rgb = normalize(vin.tangentBasis * (-lights[0].direction));
		//FragColor.rgb = N;

		FragColor.rgb = ambient + diffuse + specular;//wcolor.rgb;
		FragColor.a = 1.0;

		return;
	}else if(colorMode == 1){ // bin index
		uint binI = getBinIndex(position);
		FragColor = vec4(randomColor(binI).rgb,1.0);
		return;
	}else if(colorMode == 2){ // density
		value = density;
		minValue = 0.95 * u_rho0;
		maxValue =  1.05 * u_rho0;
	}else if(colorMode == 3){ // Temperature
		value = temp;
		minValue = 275;
		maxValue =  275+200.0;
	}else if(colorMode == 4){ // vel
		value = height;
		minValue = 0;
		maxValue =  4;
		
	}else if(colorMode == 5){ // stress
		value = stress;
		minValue = 15e3;
		maxValue =  15e4;
	}else if(colorMode == 6){ // solid color
		value = temp;
		minValue = 275;
		maxValue =  275+300.0;
	}

	vec4 wcolor = colorMap(map(value,minValue,maxValue), palette);
    
	vec3 viewDir = normalize(viewPos - position);
	vec3 lightDir = normalize(vec3(1,1,5));
    float diff = max(dot(normal, lightDir), 0.0);
    float spec = 0;
    if(diff != 0){
        vec3 halfwayDir = normalize(lightDir + viewDir);
        spec = pow(max(dot(normal, halfwayDir), 0.0), 0.1); //blinn
    }

	vec3 ambient = 0.6 * wcolor.rgb;
    vec3 diffuse = 0.4 * diff * wcolor.rgb;
    vec3 specular = 0.3 * spec * wcolor.rgb;
    //FragColor.rgb = N* 0.9 + FragColor.rgb * 0.1;
    //FragColor.rgb = normalize(vin.tangentBasis * (-lights[0].direction));
    //FragColor.rgb = N;

	FragColor.rgb = ambient + diffuse + specular;//wcolor.rgb;
    FragColor.a = 1.0;

}