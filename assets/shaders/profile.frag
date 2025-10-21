#version 430

in VS_out{
	vec4 position;
	vec4 screen_position;
	vec4 color;
	mat4 mv;
} vin;

out vec4 FragColor;

void main() {
	FragColor = vec4(1,0,0,1);
	//FragColor = vin.color;
	FragColor.w = 1.0;
}



  
