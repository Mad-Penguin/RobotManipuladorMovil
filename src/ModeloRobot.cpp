#include "ModeloRobot.h"

ModeloRobot::ModeloRobot(std::string path, int height, int width, unsigned int shader, bool show_logs) {
	// Lectura de datos
	this->path = path;
	this->height = height;
	this->width = width;
	this->shader = shader;
	this->show_logs = show_logs;
	cargaModelo();

	if (!vertices_por_partes.size()) {
		printf("[Error] El robot no tiene partes por lo que no se generaron vertex buffer ni index bufffer\n");
		return;
	}

	// Descripcion modelo DH
	rotaciones.resize(vertices_por_partes.size());
	for (int i = 0; i < rotaciones.size(); i++) rotaciones[i] = 0.0f;

	mv = glm::vec3(0.0f);

	printf("centro[0]: (%.2f, %.2f, %.2f)\n", centros[0].x, centros[0].y, centros[0].z);
	printf("centro[1]: (%.2f, %.2f, %.2f)\n", centros[1].x, centros[1].y, centros[1].z);
	printf("centro[2]: (%.2f, %.2f, %.2f)\n", centros[2].x, centros[2].y, centros[2].z);
	
	
	/*
	---------------------------------------------------------------------------------------

	Frames originales, sin aplicar DH

	---------------------------------------------------------------------------------------
	
	piezas.push_back(
		DenavitHartenberg(centros[0], glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f))
	);
	vector_gap.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
	angle_gap.push_back(0.0f);

	piezas.push_back(
		DenavitHartenberg(centros[1], glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f))
	);
	vector_gap.push_back(glm::vec3(-1.337f, 2.228f, 0.0f));
	angle_gap.push_back(54.897f);

	piezas.push_back(
		DenavitHartenberg(centros[2], glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f))
	);
	vector_gap.push_back(glm::vec3(5.495f, 8.465f, -5.680f));
	angle_gap.push_back(54.897f);

	piezas.push_back(
		DenavitHartenberg(centros[4], glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f))
	);
	vector_gap.push_back(glm::vec3(-4.010f, -3.119f, 6.262f));
	angle_gap.push_back(54.897f);
	*/

	/*
	---------------------------------------------------------------------------------------

	Frames para aplicar DH

	---------------------------------------------------------------------------------------
	*/
	vector_gap.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
	angle_gap.push_back(0.0f);

	vector_gap.push_back(glm::vec3(-1.337f, 2.228f, 0.0f));
	angle_gap.push_back(54.897f);

	vector_gap.push_back(glm::vec3(5.495f, 8.465f, -5.680f));
	angle_gap.push_back(54.897f);

	vector_gap.push_back(glm::vec3(-4.010f, -3.119f, 6.262f));
	angle_gap.push_back(54.897f);

	piezas.resize(4);
	piezas[0] = DenavitHartenberg(centros[0], glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), TIPO_ACCION::ROTATION);
	piezas[1] = DenavitHartenberg(centros[1] + vector_gap[1], glm::rotate(glm::vec3(0.0f, 0.0f, 1.0f), glm::radians(angle_gap[1]), glm::vec3(0.0f, 1.0f, 0.0f)), &(piezas[0]), TIPO_ACCION::ROTATION);
	piezas[2] = DenavitHartenberg(centros[2] + vector_gap[2], glm::rotate(glm::vec3(0.0f, 0.0f, 1.0f), glm::radians(angle_gap[2]), glm::vec3(0.0f, 1.0f, 0.0f)), &(piezas[1]), TIPO_ACCION::ROTATION);
	piezas[3] = DenavitHartenberg(centros[4] + vector_gap[3], glm::rotate(glm::vec3(0.0f, 0.0f, 1.0f), glm::radians(angle_gap[3]), glm::vec3(0.0f, 1.0f, 0.0f)), &(piezas[2]), TIPO_ACCION::ROTATION);
	
	frame_principal = {
		&(piezas[0]),
		&(piezas[0]),
		&(piezas[1]),
		&(piezas[2]),
		&(piezas[2]),
		&(piezas[3]),
		&(piezas[3]),
		&(piezas[3]),
		&(piezas[3]),
	};

	if (show_logs) {
		printf("[Info] Clases DenavitHartenberg instanciadas:\n");
		for (auto dh : piezas) {
			printf("\to: %.2f %.2f %.2f\n",dh.getO().x, dh.getO().y, dh.getO().z);
			printf("\tx: %.2f %.2f %.2f\n",dh.getX().x, dh.getX().y, dh.getX().z);
			printf("\ty: %.2f %.2f %.2f\n",dh.getY().x, dh.getY().y, dh.getY().z);
			printf("\tz: %.2f %.2f %.2f\n\n",dh.getZ().x, dh.getZ().y, dh.getZ().z);
		}
	}

	float escalamiento_ejes = 5.0f;
	unsigned int cnt = 0;
	for (auto dh : piezas) {
		glm::vec3 o = dh.getO();
		glm::vec3 eje_x = dh.getX();
		glm::vec3 eje_y = dh.getY();
		glm::vec3 eje_z = dh.getZ();

		// Eje x
		ejes.push_back(o.x);
		ejes.push_back(o.y);
		ejes.push_back(o.z);
		ejes.push_back(1.0f);
		ejes.push_back(0.0f);
		ejes.push_back(0.0f);
		ejes.push_back(0.0f);
		ejes.push_back((o + escalamiento_ejes * eje_x).x);
		ejes.push_back((o + escalamiento_ejes * eje_x).y);
		ejes.push_back((o + escalamiento_ejes * eje_x).z);
		ejes.push_back(1.0f);
		ejes.push_back(0.0f);
		ejes.push_back(0.0f);
		ejes.push_back(0.0f);

		conexion_ejes.push_back(cnt++);
		conexion_ejes.push_back(cnt++);

		// Eje y
		ejes.push_back(o.x);
		ejes.push_back(o.y);
		ejes.push_back(o.z);
		ejes.push_back(0.0f);
		ejes.push_back(1.0f);
		ejes.push_back(0.0f);
		ejes.push_back(0.0f);
		ejes.push_back((o + escalamiento_ejes * eje_y).x);
		ejes.push_back((o + escalamiento_ejes * eje_y).y);
		ejes.push_back((o + escalamiento_ejes * eje_y).z);
		ejes.push_back(0.0f);
		ejes.push_back(1.0f);
		ejes.push_back(0.0f);
		ejes.push_back(0.0f);

		conexion_ejes.push_back(cnt++);
		conexion_ejes.push_back(cnt++);

		// Eje z
		ejes.push_back(o.x);
		ejes.push_back(o.y);
		ejes.push_back(o.z);
		ejes.push_back(0.0f);
		ejes.push_back(0.0f);
		ejes.push_back(1.0f);
		ejes.push_back(0.0f);
		ejes.push_back((o + escalamiento_ejes * eje_z).x);
		ejes.push_back((o + escalamiento_ejes * eje_z).y);
		ejes.push_back((o + escalamiento_ejes * eje_z).z);
		ejes.push_back(0.0f);
		ejes.push_back(0.0f);
		ejes.push_back(1.0f);
		ejes.push_back(0.0f);

		conexion_ejes.push_back(cnt++);
		conexion_ejes.push_back(cnt++);
	}

	glGenBuffers(1, &buffer_ejes);
	glGenBuffers(1, &index_buffer_ejes);

	glBindBuffer(GL_ARRAY_BUFFER, buffer_ejes);
	glBufferData(GL_ARRAY_BUFFER, ejes.size() * sizeof(float), &(ejes[0]), GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_ejes);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, conexion_ejes.size() * sizeof(unsigned int), &(conexion_ejes[0]), GL_STATIC_DRAW);

	model_location = glGetUniformLocation(shader, "model");
	view_location = glGetUniformLocation(shader, "view");
	proj_location = glGetUniformLocation(shader, "proj");

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);

	buffers.resize(vertices_por_partes.size());
	index_buffers.resize(vertices_por_partes.size());

	if (this->show_logs)
		printf("[Info] Generando buffers\n");

	glGenBuffers(vertices_por_partes.size(), &buffers[0]);
	glGenBuffers(vertices_por_partes.size(), &index_buffers[0]);

	if (this->show_logs) {
		printf("[Info] Buffers generados\n\tBuffers: ");
		for (auto buffer : buffers) printf("%d ",buffer);
		printf("\n\tIndex buffers: ");
		for (auto index_buffer : index_buffers) printf("%d ", index_buffer);
		printf("\n");
	}

	int acum_vertices = 0, acum_faces = 0;

	if (this->show_logs)
		printf("[Info] Cargando datos en OpenGL\n");

	for (int i = 0; i < vertices_por_partes.size(); i++) {
		glBindBuffer(GL_ARRAY_BUFFER, buffers[i]);
		glBufferData(GL_ARRAY_BUFFER, 7 * vertices_por_partes[i] * sizeof(float), &vertices[7* acum_vertices], GL_STATIC_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffers[i]);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, 3 * faces_por_partes[i] * sizeof(unsigned int), &faces[3 * acum_faces], GL_STATIC_DRAW);

		acum_vertices += vertices_por_partes[i];
		acum_faces += faces_por_partes[i];
	}

	if (this->show_logs)
		printf("[Info] Carga de datos completada\n");

	glUseProgram(0);

	return;
}

// DELETE FROM HERE
void ModeloRobot::moveLastFrame(glm::vec3 d) {
	this->traslacion_frame = d;
	return;
}

void ModeloRobot::rotateLastFrame(float alpha_) {
	rota_last_frame = alpha_;
	return;
}

void ModeloRobot::printCenterOfFrame() {
	printf("[Center of last frame]\n\t%.8f %.8f %.8f\n\n", (piezas[ piezas.size() - 1 ].getO() + traslacion_frame).x, (piezas[piezas.size() - 1].getO() + traslacion_frame).y, (piezas[piezas.size() - 1].getO() + traslacion_frame).z);
}
// TO HERE

void ModeloRobot::cargaModelo() {
	std::ifstream input(path.c_str());

	if (!input.is_open()) {
		printf("[Error] No se pudo abrir el archivo con ruta: %s\n",path.c_str());
		return;
	}

	char ch;
	float x, y, z;
	bool init = false;
	int f1, f2, f3;

	int curr_cont_vertices = 0, curr_cont_faces = 0;
	int acum = 0;

	while (input >> ch) {
		switch (ch)
		{
		case 'v':
			curr_cont_vertices++;

			input >> x >> y >> z;
			vertices.push_back(x);
			vertices.push_back(y);
			vertices.push_back(z);
			vertices.push_back(0.87f);
			vertices.push_back(0.87f);
			vertices.push_back(0.87f);
			vertices.push_back(0.00f);

			if (!init) {
				min_x = max_x = x;
				min_y = max_y = y;
				min_z = max_z = z;
				init = true;
			}

			min_x = std::min(min_x, x);
			max_x = std::max(max_x, x);

			min_y = std::min(min_y, y);
			max_y = std::max(max_y, y);

			min_z = std::min(min_z, z);
			max_z = std::max(max_z, z);
			break;
		case 'f':
			curr_cont_faces++;

			input >> f1 >> f2 >> f3;
			faces.push_back(f1 - 1 - acum);
			faces.push_back(f2 - 1 - acum);
			faces.push_back(f3 - 1 - acum);
			break;
		case 's':
			if (!curr_cont_vertices) break;
			acum += curr_cont_vertices;
			vertices_por_partes.push_back(curr_cont_vertices);
			faces_por_partes.push_back(curr_cont_faces);
			curr_cont_vertices = curr_cont_faces = 0;
			break;
		default:
			if (show_logs)
				printf("[Warning] Se encontro una linea distinta a v (vertice) y f (cara).  ch: %c\n",ch);
			break;
		}
	}

	vertices_por_partes.push_back(curr_cont_vertices);
	faces_por_partes.push_back(curr_cont_faces);

	if (vertices_por_partes.size() != faces_por_partes.size())
		printf("[Error] No se tiene misma cantidad de conjunto de vertices con conjuntos de caras:\n\tvertices_por_partes: %d\n\tfaces_por_partes: %d\n", vertices_por_partes.size(), faces_por_partes.size());

	if (show_logs) {
		printf("[Info] Termino de lectura\n\tStats:\n\tPartes: %d\n\tVertices: %d\n\tCaras: %d\n\t", vertices_por_partes.size(), vertices.size() / 7, faces.size() / 3);
		printf("min_x: %.2f   max_x: %.2f\n\tmin_y: %.2f   max_y: %.2f\n\tmin_z: %.2f   max_z: %.2f\n",min_x,max_x,min_y,max_y,min_z,max_z);
	}

	centros.resize(vertices_por_partes.size());
	acum = 0;
	for (int obj_idx = 0; obj_idx < vertices_por_partes.size(); obj_idx++) {
		centros[obj_idx] = glm::vec3(0.0f);
		float curr_min_x = vertices[7 * acum], curr_max_x = vertices[7 * acum];
		float curr_min_y = vertices[7 * acum + 1], curr_max_y = vertices[7 * acum + 1];
		float curr_min_z = vertices[7 * acum + 2], curr_max_z = vertices[7 * acum + 2];
		for (int i = 0; i < vertices_por_partes[obj_idx]; i++) {
			curr_min_x = std::min(curr_min_x, vertices[7 * (i + acum)]);
			curr_max_x = std::max(curr_max_x, vertices[7 * (i + acum)]);
			
			curr_min_y = std::min(curr_min_y, vertices[7 * (i + acum) + 1]);
			curr_max_y = std::max(curr_max_y, vertices[7 * (i + acum) + 1]);
			
			curr_min_z = std::min(curr_min_z, vertices[7 * (i + acum) + 2]);
			curr_max_z = std::max(curr_max_z, vertices[7 * (i + acum) + 2]);
			/*centros[obj_idx] += glm::vec3(
				vertices[7 * (i + acum)],
				vertices[7 * (i + acum) + 1],
				vertices[7 * (i + acum) + 2]
			);*/
		}
		//centros[obj_idx] /= (float)(vertices_por_partes[obj_idx]);
		centros[obj_idx] = glm::vec3(
			(curr_min_x + curr_max_x) / 2.0f,
			(curr_min_y + curr_max_y) / 2.0f,
			(curr_min_z + curr_max_z) / 2.0f
		);
		acum += vertices_por_partes[obj_idx];
	}

	return;
}

void ModeloRobot::muestraEjes(bool flag) {
	this->muestra_ejes = flag;
	return;
}

void ModeloRobot::traslada(glm::vec3 mv) {
	this->mv = glm::vec3(mv.z, mv.x, mv.y);
	return;
}

void ModeloRobot::rotaBase(float alpha_) {
	//rotaciones[1] = alpha_;
	piezas[0].rota(alpha_);
	return;
}

void ModeloRobot::rotaBrazo1(float alpha_) {
	piezas[1].rota(alpha_);
	return;
}
void ModeloRobot::rotaBrazo2(float alpha_) {
	piezas[2].rota(alpha_);
	return;
}
void ModeloRobot::rotaPinza(float alpha_) {
	piezas[3].rota(alpha_);
	return;
}

void ModeloRobot::dibujaRobot() {

	//glm::mat4 view(1.0f);
	
	//view = translateMat4(view, 0.0f, 0.0f, -4.0f);
	// 
	// Este estaba antes!
	// view = glm::translate(view, glm::vec3(0.0f, 0.0f, -min_z - min_z));

	glm::mat4 view = glm::lookAt(
		glm::vec3(0.0f, 35.0f, min_z - 100.0f), // eye
		glm::vec3(0.0f, 35.0f, 0.0f),		 // center
		glm::vec3(0.0f, 1.0f, 0.0f)			 // up
	);

	//glm::mat4 proj = glm::ortho(-35.0f, 35.0f, -20.0f, 20.0f, -20.0f, 20.0f);
	//glm::mat4 proj = glm::ortho( -2.0f*(max_x - min_x), 2.0f * (max_x - min_x), -2.0f * (max_y - min_y), 2.0f * (max_y - min_y), -2.0f * (max_z - min_z), 2.0f * (max_z - min_z));
	
	// Este estaba antes!
	/*glm::mat4 proj = glm::ortho(
		-0.875f*2.0f * (max_y - min_y), 0.875f * 2.0f * (max_y - min_y), 
		-(max_y - min_y), (max_y - min_y), 
		//-(max_z - min_z),  (max_z - min_z)
		-0.875f * 2.0f * (max_y - min_y), 0.875f * 2.0f * (max_y - min_y)
	);*/

	glm::mat4 proj = glm::perspective(45.0f, (GLfloat)height / (GLfloat)width, 1.0f, 150.0f);
	//glm::mat4 proj = glm::perspective(glm::radians(alpha), (float)(height) / (float)(width), 0.1f, 100.0f);
	
	/*printf("%.2f\n", alpha);
	alpha += d_alpha;

	if (alpha > 10.0f) {
		alpha = 0.0f;
		printf("Todo!");
		getchar();
	}
	*/
	
	glm::mat4 model = glm::mat4(1.0f);
	//model = glm::rotate(model, glm::radians(rotaciones[1]), glm::vec3(0.0f, 1.0f, 0.0f));
	model = glm::translate(model, -centros[0]);

	for (int i = 0; i < how_many_parts /*vertices_por_partes.size()*/; i++) {
		// Dibujando i-esima parte
		glBindBuffer(GL_ARRAY_BUFFER, buffers[i]);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffers[i]);

		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), 0);

		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (const void*)(3 * sizeof(float)));

		model = frame_principal[i]->getModel(false);

		glUniformMatrix4fv(view_location, 1, GL_FALSE, glm::value_ptr(view));
		glUniformMatrix4fv(proj_location, 1, GL_FALSE, glm::value_ptr(proj));
		glUniformMatrix4fv(model_location, 1, GL_FALSE, glm::value_ptr(model));
		/*
		if (i == 0) {
			piezas[0].traslada(this->mv);
			//piezas[0].rota(rotaciones[0]);
			glUniformMatrix4fv(model_location, 1, GL_FALSE, glm::value_ptr(piezas[0].getModel()));
		}
		if (i == 1) {
			//piezas[1].traslada(glm::vec3(0.0f));
			piezas[1].rota(rotaciones[1]);
			glUniformMatrix4fv(model_location, 1, GL_FALSE, glm::value_ptr(piezas[1].getModel()));
		}
		*/
		glDrawElements(GL_TRIANGLES, 3*faces_por_partes[i], GL_UNSIGNED_INT, 0);
	}

	//getchar();

	if (muestra_ejes) {
		for (int i = 0; i < piezas.size(); i++) {
			float __extra__ = 0.0f;
			if (i + 1 >= piezas.size()) __extra__ = rota_last_frame;
			model = glm::rotate(glm::mat4(1.0f), glm::radians(rotaciones[1]), glm::vec3(0.0f, 1.0f, 0.0f));

			glm::vec3 __extra_v__(0.0f);
			if (i + 1 >= piezas.size()) __extra_v__ = traslacion_frame;
			model = glm::translate(model, -centros[0] + vector_gap[i] + __extra_v__);

			model = glm::translate(model, piezas[i].getO());
			model = glm::rotate(model, glm::radians(angle_gap[i] + __extra__), glm::vec3(0.0f, 1.0f, 0.0f));
			model = glm::translate(model, -piezas[i].getO());

			// Eliminar esto para aplicar vector gap y angle gap
			//model = glm::rotate(glm::mat4(1.0f), glm::radians(rotaciones[1]), glm::vec3(0.0f, 1.0f, 0.0f));
			model = glm::translate(glm::mat4(1.0f), -centros[0]);


			glBindBuffer(GL_ARRAY_BUFFER, buffer_ejes);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_ejes);

			glEnableVertexAttribArray(0);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), 0);

			glEnableVertexAttribArray(1);
			glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (const void*)(3 * sizeof(float)));

			glUniformMatrix4fv(view_location, 1, GL_FALSE, glm::value_ptr(view));
			glUniformMatrix4fv(proj_location, 1, GL_FALSE, glm::value_ptr(proj));
			glUniformMatrix4fv(model_location, 1, GL_FALSE, glm::value_ptr(model));

			glLineWidth(2.0f);
			glDrawElements(GL_LINES, conexiones_por_frame, GL_UNSIGNED_INT, (const void*)(i * conexiones_por_frame * sizeof(unsigned int)));
		}

		/*model = glm::rotate(glm::mat4(1.0f), glm::radians(rotaciones[1]), glm::vec3(0.0f, 1.0f, 0.0f));
		model = glm::translate(model, -centros[0]);
		
		glBindBuffer(GL_ARRAY_BUFFER, buffer_ejes);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_ejes);

		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), 0);

		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (const void*)(3 * sizeof(float)));

		glUniformMatrix4fv(view_location, 1, GL_FALSE, glm::value_ptr(view));
		glUniformMatrix4fv(proj_location, 1, GL_FALSE, glm::value_ptr(proj));
		glUniformMatrix4fv(model_location, 1, GL_FALSE, glm::value_ptr(model));

		glLineWidth(2.0f);
		glDrawElements(GL_LINES, conexion_ejes.size() - conexiones_last_frame, GL_UNSIGNED_INT, 0);

		model = glm::rotate(glm::mat4(1.0f), glm::radians(rotaciones[1] + rota_last_frame), glm::vec3(0.0f, 1.0f, 0.0f));
		model = glm::translate(model, -centros[0]);
		model = glm::translate(model, this->traslacion_frame);
		glUniformMatrix4fv(model_location, 1, GL_FALSE, glm::value_ptr(model));

		glLineWidth(2.0f);
		glDrawElements(GL_LINES, conexiones_last_frame, GL_UNSIGNED_INT, (const void*)((conexion_ejes.size() - conexiones_last_frame) * sizeof(unsigned int)) );*/
	}

	return;
}