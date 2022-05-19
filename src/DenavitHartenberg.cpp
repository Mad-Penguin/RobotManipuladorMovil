#include "DenavitHartenberg.h"


DenavitHartenberg::DenavitHartenberg(glm::vec3 o, glm::vec3 x, glm::vec3 y, glm::vec3 z, TIPO_ACCION tipo) {
	this->o = this->origin_o = o;
	this->x = x / norm(x);
	this->y = y / norm(y);
	this->z = z / norm(z);
	this->tipo = tipo;
	this->anterior = nullptr;
	return;
}

float DenavitHartenberg::norm(glm::vec3 v) {
	return glm::sqrt(glm::dot(v, v));
}

DenavitHartenberg::DenavitHartenberg(glm::vec3 por_donde_pasa_z, glm::vec3 z, DenavitHartenberg* anterior, TIPO_ACCION tipo) {
	this->z = z / norm(z);
	this->anterior = anterior;
	this->tipo = tipo;
	this->origin_o = por_donde_pasa_z;

	glm::vec3 p1 = por_donde_pasa_z, p2 = por_donde_pasa_z + z;
	glm::vec3 p3 = anterior->o, p4 = anterior->o + anterior->z;
	glm::vec3 pa, pb;
	float mua, mub;

	if (lineLineIntersect(p1, p2, p3, p4, &pa, &pb, &mua, &mub)) {
		if(norm(pa - pb) < epsilon) {
			// El punto de ambas lineas coinciden por lo que se intersectan
			this->o = pa;
			this->x = glm::cross(p1 - pa, p3 - pa);	// El eje x va a ser la normal al plano
			this->x /= norm(this->x);
		}
		else {
			// Los puntos son distintos por lo que son los que inducen la distancia minima, las lineas no son coplanares
			this->o = pa;
			this->x = (pb - pa) / norm(pb - pa);
		}
	}
	else {
		// Paralelas
		float d = norm(glm::cross(p3 - p1, p3 - p2)) / norm( p2 - p1 );
		float h = norm(p1 - p3);

		float alpha = sqrt(h*h - d*d);

		glm::vec3 dir_1 = p1 + alpha * this->z - p3;
		glm::vec3 dir_2 = p1 - alpha * this->z - p3;
		glm::vec3 dir;
		if (fabs(glm::dot(this->z, dir_1)) < epsilon) {
			dir = dir_1 / norm(dir_1);
		}
		else {
			dir = dir_2 / norm(dir_2);
		}

		this->o = p3 + d * dir;
		this->x = -dir;
	}

	this->y = glm::cross(this->z, this->x);
	this->y /= norm(this->y);

	return;
}
/*
---------------------------------------------------------------------------------------

		Algoritmo para encontrar la distancia minima entre dos lineas en R^3

Autor:  Paul Bourke
Teoria: http://paulbourke.net/geometry/pointlineplane/
Codigo fuente: http://paulbourke.net/geometry/pointlineplane/lineline.c

---------------------------------------------------------------------------------------
*/
bool DenavitHartenberg::lineLineIntersect(
	glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4, glm::vec3* pa, glm::vec3* pb,
	float* mua, float* mub){
	glm::vec3 p13, p43, p21;
	double d1343, d4321, d1321, d4343, d2121;
	double numer, denom;

	p13.x = p1.x - p3.x;
	p13.y = p1.y - p3.y;
	p13.z = p1.z - p3.z;
	p43.x = p4.x - p3.x;
	p43.y = p4.y - p3.y;
	p43.z = p4.z - p3.z;
	if (fabs(p43.x) < epsilon && fabs(p43.y) < epsilon && fabs(p43.z) < epsilon)
		return false;
	p21.x = p2.x - p1.x;
	p21.y = p2.y - p1.y;
	p21.z = p2.z - p1.z;
	if (fabs(p21.x) < epsilon && fabs(p21.y) < epsilon && fabs(p21.z) < epsilon)
		return false;

	d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
	d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
	d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
	d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
	d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

	denom = d2121 * d4343 - d4321 * d4321;
	if (fabs(denom) < epsilon)
		return false;
	numer = d1343 * d4321 - d1321 * d4343;

	*mua = numer / denom;
	*mub = (d1343 + d4321 * (*mua)) / d4343;

	pa->x = p1.x + *mua * p21.x;
	pa->y = p1.y + *mua * p21.y;
	pa->z = p1.z + *mua * p21.z;
	pb->x = p3.x + *mub * p43.x;
	pb->y = p3.y + *mub * p43.y;
	pb->z = p3.z + *mub * p43.z;

	return true;
}

void DenavitHartenberg::rota(float alpha) {
	if (this->tipo == TIPO_ACCION::PRISMATIC) {
		printf("[Warning] Se esta intentando rotar un eje declarado como prismatic, si bien esto no tendra efecto debes checar el codigo\n");
		return;
	}
	this->alpha = alpha;
	return;
}

void DenavitHartenberg::traslada(float mv) {
	if (this->tipo == TIPO_ACCION::ROTATION) {
		printf("[Warning] Se esta intentando trasladar un eje declarado como rotacion, si bien esto no tendra efecto debes checar el codigo\n");
		return;
	}
	this->mv = mv;
	return;
}

glm::mat4 DenavitHartenberg::getModel(bool memo, bool call_from_frame) {
	// Evitar recalcular
	if (call_from_frame && memo) return this->model;

	switch (this->tipo) {
	case TIPO_ACCION::ROTATION:
		this->model = glm::translate(glm::mat4(1.0f), this->origin_o);
		this->model = glm::rotate(this->model, glm::radians(this->alpha), this->z);
		this->model = glm::translate(this->model, -this->origin_o);
		break;
	case TIPO_ACCION::PRISMATIC:
		this->model = glm::translate(glm::mat4(1.0f), this->mv * this->z);
		break;
	default:
		this->model = glm::mat4(1.0f);
		printf("[Critical] Por alguna razon no esta definido el tipo de accion de este eje, checar como paso el constructor\n");
	}

	if(this->anterior) this->model = this->anterior->getModel( this->anterior->getTipo() == TIPO_ACCION::ROTATION) * this->model;
	return this->model;
}

TIPO_ACCION DenavitHartenberg::getTipo() {
	return this->tipo;
}

glm::vec3 DenavitHartenberg::getO() {
	return this->o;
}
glm::vec3 DenavitHartenberg::getX() {
	return this->x;
}
glm::vec3 DenavitHartenberg::getY() {
	return this->y;
}
glm::vec3 DenavitHartenberg::getZ() {
	return this->z;
}

float DenavitHartenberg::getParametro() {
	if (this->tipo == TIPO_ACCION::PRISMATIC) return this->mv;
	return this->alpha;
}