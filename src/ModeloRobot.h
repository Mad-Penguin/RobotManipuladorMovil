#pragma once
#include "DenavitHartenberg.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include "PQP.h"


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

	std::vector< DenavitHartenberg* > frame_principal;

	/*
		-------- Rangos de los grados de libertad --------
		Base: 		[-180, 180]
		Brazo 1:	[-60, 145]
		Brazo 2:	[-180, 0]
		Pinzas:		[-78, -6]
	*/
	float min_angle_base = -180.0f, max_angle_base = 180.0f;
	float min_angle_brazo1 = -60.0f, max_angle_brazo1 = 145.0f;
	float min_angle_brazo2 = -180.0f, max_angle_brazo2 = 0.0f;
	float min_angle_pinzas = -78.0f, max_angle_pinzas = -6.0f;
	
	// OpenGL
	int height, width;
	unsigned int shader;
	std::vector<unsigned int> buffers, index_buffers;
	int model_location, view_location, proj_location;

	unsigned int buffer_ejes, index_buffer_ejes;

	float alpha = 0.0f, d_alpha = 0.5f;

	// Camara
	float theta = 0.0f, phi = 0.0f, radio = 300.0f;
	glm::vec3 eye, center = glm::vec3(0.0f, 35.0f, 0.0f), up = glm::vec3(0.0f, 1.0f, 0.0f);

	// Denavit Hartenberg
	std::vector< DenavitHartenberg > piezas;
	std::vector< glm::vec3 > vector_gap;
	std::vector< float > angle_gap;
	std::vector< float > rotaciones;
	glm::vec3 mv;

	// Frames
	std::vector<float> ejes;
	std::vector<unsigned int> conexion_ejes;
	unsigned int conexiones_por_frame = 6;

	// Banderas
	bool show_logs, muestra_ejes;

	// DELETE THIS
	glm::vec3 traslacion_frame = glm::vec3(0.0f);
	unsigned int conexiones_last_frame = 6;
	float rota_last_frame = 0.0f;

	glm::vec3 updateEye();

	// PQP
	std::vector< PQP_Model* > PQP_Models;
	std::vector< std::pair<unsigned int, unsigned int> > posible_autointerseccion;
	void buildCollisionModels();

	bool enColision();

public:
	// DELETE THIS
	int how_many_parts;
	void moveLastFrame(glm::vec3 d);
	void rotateLastFrame(float alpha_);
	void printCenterOfFrame();

	ModeloRobot(std::string path, int height, int width, unsigned int shader, bool show_logs = false);

	bool trasladaEje1(float mv);
	bool trasladaEje2(float mv);
	bool rotaBase(float alpha);
	bool rotaBrazo1(float alpha);
	bool rotaBrazo2(float alpha);
	bool rotaPinza(float alpha);

	void rotaCamara(float theta_, float phi_);

	void dibujaRobot();
	void muestraEjes(bool flag);
};