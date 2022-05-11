#pragma once
#include "DenavitHartenberg.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>


class ModeloRobot {
private:
	// Descripcion del robot
	std::string path;
	void cargaModelo();

	std::vector<float> vertices;
	std::vector<unsigned int> faces;

	std::vector<unsigned int> vertices_por_partes;
	std::vector<unsigned int> faces_por_partes;

	float min_x, max_x;
	float min_y, max_y;
	float min_z, max_z;

	std::vector<glm::vec3> centros;
	
	// OpenGL
	int height, width;
	unsigned int shader;
	std::vector<unsigned int> buffers, index_buffers;
	int model_location, view_location, proj_location;

	std::vector<float> ejes;
	std::vector<unsigned int> conexion_ejes;
	unsigned int buffer_ejes, index_buffer_ejes;

	float alpha = 0.0f, d_alpha = 0.5f;

	// Denavit Hartenberg
	std::vector< DenavitHartenberg > piezas;
	std::vector< float > rotaciones;
	glm::vec3 mv;

	// Banderas
	bool show_logs, muestra_ejes;

	// DELETE THIS
	glm::vec3 traslacion_frame = glm::vec3(0.0f);
	unsigned int conexiones_last_frame = 6;

public:
	// DELETE THIS
	int how_many_parts;
	void moveLastFrame(glm::vec3 d);
	void printCenterOfFrame();

	ModeloRobot(std::string path, int height, int width, unsigned int shader, bool show_logs = false);

	void traslada(glm::vec3 mv);
	void rotaBase(float alpha);

	void dibujaRobot();
	void muestraEjes(bool flag);
};