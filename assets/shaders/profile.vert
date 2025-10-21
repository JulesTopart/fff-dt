#version 430
#include "../common/shaders/graphics/colors.comp"

layout (location = 0) in vec3 _position;
layout (location = 1) in vec3 _normal;
layout (location = 2) in vec3 _color;

out VS_out{
	vec4 position;
	vec4 screen_position;
	vec4 color;
	mat4 mv;
} vout;

uniform mat4 view;
uniform mat4 projection;
uniform mat4 model;

void main() {

	vout.position = model * (vec4(_position,1));
	vout.screen_position = projection * view * vout.position;

	vout.color = vec4(_color,1);
	vout.mv = projection * view;
		
	gl_Position = vout.screen_position;	
}
