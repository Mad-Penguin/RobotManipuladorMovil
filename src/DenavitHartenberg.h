#pragma once
#include <glm.hpp>
#include <gtc/type_ptr.hpp>
#include <gtx/rotate_vector.hpp>

enum class TIPO_ACCION {
	ROTATION = 0,
	PRISMATIC = 1
};

class DenavitHartenberg {
private:
	// Convencion DenavitHartenberg
	glm::vec3 origin_o, o, x, y, z;
	DenavitHartenberg *anterior = nullptr;
	TIPO_ACCION tipo;

	glm::mat4 model = glm::mat4(1.0);

	// Variables de movimiento rotacion/traslacion
	float alpha = 0.0f;
	float mv = 0.0f;

	// Cosas auxiliares
	float epsilon = 0.001f;
	bool lineLineIntersect(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4, glm::vec3 *pa, glm::vec3 *pb, float* mua, float* mub);

public:
	// Constructor default
	DenavitHartenberg(){}
	// Constructor para el primer eslabon
	DenavitHartenberg(glm::vec3 o, glm::vec3 x, glm::vec3 y, glm::vec3 z, TIPO_ACCION tipo);
	// Constructor para el segundo eslabon en adelante
	DenavitHartenberg(glm::vec3 por_donde_pasa_z, glm::vec3 z, DenavitHartenberg* anterior, TIPO_ACCION tipo);

	float norm(glm::vec3 v);
	
	void rota(float alha);
	void traslada(float mv);

	TIPO_ACCION getTipo();
	glm::vec3 getO();
	glm::vec3 getX();
	glm::vec3 getY();
	glm::vec3 getZ();
	glm::mat4 getModel(bool memo, bool call_from_frame = false);

	float getParametro();

};