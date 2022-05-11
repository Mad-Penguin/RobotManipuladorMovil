#shader vertex
#version 410 core

layout(location = 0) in vec4 position;
layout(location = 1) in vec4 colorVertex_VertexShader;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

out vec4 colorVertex_FragmentShader;

void main() {
	colorVertex_FragmentShader = colorVertex_VertexShader;
	gl_Position = proj * view * model * position;
};

#shader fragment
#version 410 core

layout(location = 0) out vec4 color;

uniform vec4 u_Color;
in vec4 colorVertex_FragmentShader;

void main() {
	color = colorVertex_FragmentShader;
};