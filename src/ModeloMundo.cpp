#include "ModeloMundo.h"

ModeloMundo::ModeloMundo(MUNDOS _tipo_de_mundo_, int _height_, int _width_, unsigned int _shader_) {
	tipo_de_mundo = _tipo_de_mundo_;
    if (tipo_de_mundo == MUNDOS::MUNDO_1) {
        idx_mundo = 0;
        vertices = &mundo_1_vertices;
        relacion_vertices = &mundo_1_relaciones;
    }
    if (tipo_de_mundo == MUNDOS::MUNDO_2) {
        idx_mundo = 1;
        vertices = &mundo_2_vertices;
        relacion_vertices = &mundo_2_relaciones;
    }
    if (tipo_de_mundo == MUNDOS::MUNDO_3) {
        idx_mundo = 2;
        vertices = &mundo_3_vertices;
        relacion_vertices = &mundo_3_relaciones;
    }
    if (tipo_de_mundo == MUNDOS::MUNDO_4) {
        idx_mundo = 3;
        vertices = &mundo_4_vertices;
        relacion_vertices = &mundo_4_relaciones;
    }

    pos_robot = real_pos_robot = glm::vec3((*vertices)[0], (*vertices)[1], 0.0f);

	height = _height_;
	width = _width_;
	shader = _shader_;

    model_location = glGetUniformLocation(shader, "model");
    view_location = glGetUniformLocation(shader, "view");
    proj_location = glGetUniformLocation(shader, "proj");
	
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    glGenBuffers(1, &buffer);
    glBindBuffer(GL_ARRAY_BUFFER, buffer);
    glBufferData(GL_ARRAY_BUFFER, (*vertices).size() * sizeof(float), &((*vertices)[0]), GL_DYNAMIC_DRAW);

    glGenBuffers(1, &index_buffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, num_indices[idx_mundo] * sizeof(unsigned int), &((*relacion_vertices)[0]), GL_DYNAMIC_DRAW);

    glGenBuffers(1, &buffer_rvg);
    glGenBuffers(1, &index_buffer_rvg_vertices);
    glGenBuffers(1, &index_buffer_rvg_aristas);
    
    glGenBuffers(1, &buffer_camino);
    glGenBuffers(1, &index_buffer_camino);

    glUseProgram(0);

    calculaRVG();
    return;
}

void ModeloMundo::dibujaRVG() {
    if (!muestra_rvg) return;
    glBindBuffer(GL_ARRAY_BUFFER, buffer_rvg);
    glBufferData(GL_ARRAY_BUFFER, rvg_vertices.size() * sizeof(float), &(rvg_vertices[0]), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float), 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_rvg_vertices);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, relacion_rvg_vertices.size() * sizeof(unsigned int), &(relacion_rvg_vertices[0]), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (const void*)(2 * sizeof(float)));
    glPointSize(7.5f);
    glDrawElements(GL_POINTS, relacion_rvg_vertices.size(), GL_UNSIGNED_INT, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_rvg_aristas);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, relacion_rvg_aristas.size() * sizeof(unsigned int), &(relacion_rvg_aristas[0]), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (const void*)(2 * sizeof(float)));
    

    glDrawElements(GL_LINES, relacion_rvg_aristas.size(), GL_UNSIGNED_INT, 0);
    return;
}

void ModeloMundo::genCamino() {
    if (!need_update) return;

    camino_vertices.clear();
    relacion_camino_aristas.clear();

    /// Priority queue que ordena por menor distancia y contiene informacion de la ultima arista que se uso
    std::priority_queue<
        std::pair<float, std::pair<int,int>>,
        std::vector<std::pair<float, std::pair<int, int>>>,
        std::greater<std::pair<float, std::pair<int, int>>>
    > pq;
    glm::vec3 ini((*vertices)[0], (*vertices)[1], 0.0f), fin((*vertices)[6], (*vertices)[7], 0.0f);

    bool existe_colision = false;
    calculaColisionMasCercanaSinExtremos(ini, fin, &existe_colision);

    if (!existe_colision) {
        camino_vertices.push_back(ini.x);
        camino_vertices.push_back(ini.y);
        camino_vertices.push_back(1.0f);
        camino_vertices.push_back(1.0f);
        camino_vertices.push_back(1.0f);
        camino_vertices.push_back(0.0f);
        
        camino_vertices.push_back(fin.x);
        camino_vertices.push_back(fin.y);
        camino_vertices.push_back(1.0f);
        camino_vertices.push_back(1.0f);
        camino_vertices.push_back(1.0f);
        camino_vertices.push_back(0.0f);

        relacion_camino_aristas.push_back(0);
        relacion_camino_aristas.push_back(1);
        return;
    }

    for (int id = 0; id < cnt_vertices; id++) {
        glm::vec3 v = id_to_vertices[id];
        existe_colision = false;
        calculaColisionMasCercanaSinExtremos(ini, v, &existe_colision);

        if (!existe_colision) {
            pq.push({ norm(v - ini), {id , -2} });
        }
    }

    std::vector<bool> use(cnt_vertices, false);
    std::vector<int> comes_from(cnt_vertices, -3);
    int fin_comes_from = -3;

    while (!pq.empty()) {
        int id_curr = pq.top().second.first, id_prev = pq.top().second.second;
        float curr_dist = pq.top().first;
        pq.pop();

        if (id_curr == -1) {
            fin_comes_from = id_prev;
            break;
        }

        if (use[id_curr]) continue;
        comes_from[id_curr] = id_prev;
        use[id_curr] = true;

        for (auto id_nxt : rvg[id_curr]) {
            if (use[id_nxt]) continue;
            pq.push({ curr_dist + norm(id_to_vertices[id_nxt] - id_to_vertices[id_curr]), {id_nxt, id_curr}});
        }

        existe_colision = false;
        calculaColisionMasCercanaSinExtremos(id_to_vertices[id_curr], fin, &existe_colision);
        if (!existe_colision) {
            pq.push({ curr_dist + norm(fin - id_to_vertices[id_curr]), {-1, id_curr} });
        }
    }

    camino_vertices.push_back(fin.x);
    camino_vertices.push_back(fin.y);
    camino_vertices.push_back(1.0f);
    camino_vertices.push_back(1.0f);
    camino_vertices.push_back(1.0f);
    camino_vertices.push_back(0.0f);

    camino_vertices.push_back(id_to_vertices[fin_comes_from].x);
    camino_vertices.push_back(id_to_vertices[fin_comes_from].y);
    camino_vertices.push_back(1.0f);
    camino_vertices.push_back(1.0f);
    camino_vertices.push_back(1.0f);
    camino_vertices.push_back(0.0f);

    relacion_camino_aristas.push_back(0);
    relacion_camino_aristas.push_back(1);

    int curr_vertex_reconstruction = fin_comes_from, cnt_relacion = 2;
    while (curr_vertex_reconstruction != -2) {
        camino_vertices.push_back(id_to_vertices[curr_vertex_reconstruction].x);
        camino_vertices.push_back(id_to_vertices[curr_vertex_reconstruction].y);
        camino_vertices.push_back(1.0f);
        camino_vertices.push_back(1.0f);
        camino_vertices.push_back(1.0f);
        camino_vertices.push_back(0.0f);

        glm::vec3 prev_vertex_reconstruction;
        if (comes_from[curr_vertex_reconstruction] == -2) {
            prev_vertex_reconstruction = ini;
        }
        else {
            prev_vertex_reconstruction = id_to_vertices[ comes_from[curr_vertex_reconstruction] ];
        }

        camino_vertices.push_back(prev_vertex_reconstruction.x);
        camino_vertices.push_back(prev_vertex_reconstruction.y);
        camino_vertices.push_back(1.0f);
        camino_vertices.push_back(1.0f);
        camino_vertices.push_back(1.0f);
        camino_vertices.push_back(0.0f);

        relacion_camino_aristas.push_back(cnt_relacion++);
        relacion_camino_aristas.push_back(cnt_relacion++);

        curr_vertex_reconstruction = comes_from[curr_vertex_reconstruction];
    }

    return;
}

void ModeloMundo::dibujaCamino() {
    if (!muestra_camino) return;

    genCamino();
    
    glBindBuffer(GL_ARRAY_BUFFER, buffer_camino);
    glBufferData(GL_ARRAY_BUFFER, camino_vertices.size() * sizeof(float), &(camino_vertices[0]), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float), 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_camino);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, relacion_camino_aristas.size() * sizeof(unsigned int), &(relacion_camino_aristas[0]), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (const void*)(2 * sizeof(float)));
    
    glLineWidth(3.25f);

    glDrawElements(GL_LINES, relacion_camino_aristas.size(), GL_UNSIGNED_INT, 0);

    return;
}

void ModeloMundo::dibujaMundo() {
    glm::mat4 view(1.0f);
    glm::mat4 proj = glm::ortho(-3.5f, 3.5f, -2.0f, 2.0f, -2.0f, 2.0f);
    glm::mat4 model(1.0f);

    glGenBuffers(1, &buffer);
    glBindBuffer(GL_ARRAY_BUFFER, buffer);
    glBufferData(GL_ARRAY_BUFFER, (*vertices).size() * sizeof(float), &((*vertices)[0]), GL_DYNAMIC_DRAW);

    glGenBuffers(1, &index_buffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, num_indices[idx_mundo] * sizeof(unsigned int), &((*relacion_vertices)[0]), GL_DYNAMIC_DRAW);

    glUniformMatrix4fv(view_location, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(proj_location, 1, GL_FALSE, glm::value_ptr(proj));
    glUniformMatrix4fv(model_location, 1, GL_FALSE, glm::value_ptr(model));

    glBindBuffer(GL_ARRAY_BUFFER, buffer);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float), 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (const void*)(2 * sizeof(float)));
    glPointSize(5.0f);
    glLineWidth(2.5f);

    glDrawElements(GL_LINE_LOOP, num_vertices_borde[idx_mundo], GL_UNSIGNED_INT, (const void*)(2 * sizeof(unsigned int)));

    model = glm::translate(model, movimiento_acumulado_robot);
    glUniformMatrix4fv(model_location, 1, GL_FALSE, glm::value_ptr(model));
    glDrawElements(GL_POINTS, 2, GL_UNSIGNED_INT, 0);

    model = glm::mat4(1.0f);
    glUniformMatrix4fv(model_location, 1, GL_FALSE, glm::value_ptr(model));

    // Dibujando obstaculos (en caso de que existan)
    if (num_indices[idx_mundo] > num_vertices_borde[idx_mundo] + 1) {
        glDrawElements(GL_TRIANGLES, num_indices[idx_mundo] - num_vertices_borde[idx_mundo] - 2, GL_UNSIGNED_INT, (const void*)((num_vertices_borde[idx_mundo] + 2) * sizeof(unsigned int)));
    }

    // Dibujar RVG
    dibujaRVG();
    dibujaCamino();

	return;
}

void ModeloMundo::asignaPuntoInicial(float x, float y) {
    need_update = true;
    float real_x, real_y;
    real_x = -3.5f + 7.0f * (x / height);
    real_y = 2.0f - 4.0f * (y / width);
    (*vertices)[0] = real_x;
    (*vertices)[1] = real_y;
    return;
}

void ModeloMundo::asignaPuntoFinal(float x, float y) {
    need_update = true;
    float real_x, real_y;
    real_x = -3.5f + 7.0f * (x / height);
    real_y = 2.0f - 4.0f * (y / width);
    (*vertices)[6] = real_x;
    (*vertices)[7] = real_y;
    return;
}

float ModeloMundo::norm(glm::vec3 v) {
    return glm::sqrt(glm::dot(v,v));
}

void ModeloMundo::muestraRVG(bool flag) {
    muestra_rvg = flag;
    return;
}

void ModeloMundo::muestraCamino(bool flag) {
    muestra_camino = flag;
    return;
}

bool ModeloMundo::enSegmento(glm::vec3 p, glm::vec3 q, glm::vec3 r) {
    if (q[0] <= std::max(p[0], r[0]) + epsilon && q[0] + epsilon >= std::min(p[0], r[0]) &&
        q[1] <= std::max(p[1], r[1]) + epsilon && q[1] + epsilon >= std::min(p[1], r[1]))
        return true;

    return false;
}

int ModeloMundo::orientacion(glm::vec3 p, glm::vec3 q, glm::vec3 r){   
    float val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1]);
    if (abs(val) < epsilon) return 0;  // collinear
    return (val > 0.0f) ? 1 : 2; // clock or counterclock wise
}

int ModeloMundo::orientacionBinariaNoColineal(glm::vec3 p, glm::vec3 q, glm::vec3 r) {
    return 2-orientacion(p, q, r);
}

bool ModeloMundo::segmentoSegmentoInterseccion(glm::vec3 A, glm::vec3 B, glm::vec3 C, glm::vec3 D, glm::vec3* colision_mas_cercana) {
    int o1 = orientacion(A, B, C);
    int o2 = orientacion(A, B, D);
    int o3 = orientacion(C, D, A);
    int o4 = orientacion(C, D, B);

    if (colision_mas_cercana != nullptr) (*colision_mas_cercana)[2] = 0.0f;

    if (o1 != o2 && o3 != o4 && o1 != 0 && o2 != 0 && o3 != 0 && o4 != 0) {
        if (colision_mas_cercana != nullptr) {
            if ( abs(A[0] - B[0]) < epsilon ) {
                float m2 = (D[1] - C[1]) / (D[0] - C[0]);
                (*colision_mas_cercana)[0] = A[0];
                (*colision_mas_cercana)[1] = m2*(A[0]-C[0]) + C[1];
            }
            else if ( abs(C[0] - D[0]) < epsilon ) {
                float m1 = (B[1] - A[1]) / (B[0] - A[0]);
                (*colision_mas_cercana)[0] = C[0];
                (*colision_mas_cercana)[1] = m1 * (C[0] - A[0]) + A[1];
            }
            else {
                float m1 = (B[1] - A[1]) / (B[0] - A[0]);
                float m2 = (D[1] - C[1]) / (D[0] - C[0]);

                (*colision_mas_cercana)[0] = (m1 * A[0] - A[1] - m2 * C[0] + C[1]) / (m1 - m2);
                (*colision_mas_cercana)[1] = m1 * ((*colision_mas_cercana)[0] - A[0]) + A[1];
            }
        }
        return true;
    }

    bool ans = false;
    float min_dist = -1.0f;

    if (o1 == 0 && enSegmento(A, C, B)) {
        if (colision_mas_cercana != nullptr) {
            if (min_dist == -1.0f || norm(C - A) < min_dist) {
                min_dist = norm(C - A);
                *colision_mas_cercana = C;
            }
        }
        ans = true;
    }

    if (o2 == 0 && enSegmento(A, D, B)) {
        if (colision_mas_cercana != nullptr) {
            if (min_dist == -1.0f || norm(D - A) < min_dist) {
                min_dist = norm(D - A);
                (*colision_mas_cercana) = D;
            }
        }
        ans = true;
    }

    if (o3 == 0 && enSegmento(C, A, D)) {
        if (colision_mas_cercana != nullptr) {
            if (min_dist == -1.0f || norm(D - A) < min_dist) {
                min_dist = 0.0f;
                (*colision_mas_cercana) = A;
            }
        }
        ans = true;
    }

    if (o4 == 0 && enSegmento(C, B, D)) {
        if (colision_mas_cercana != nullptr) {
            if (min_dist == -1.0f || norm(B - A) < min_dist) {
                min_dist = norm(B - A);
                *colision_mas_cercana = B;
            }
        }
        ans = true;
    }
    return ans;
}

glm::vec3 ModeloMundo::calculaColisionMasCercanaSinExtremos(glm::vec3 A, glm::vec3 B, bool* existe_colision) {
    glm::vec3 d = (B - A) / norm(B - A);
    glm::vec3 ini = A + 0.01f * d, fin = B - 0.01f * d;
    return calculaColisionMasCercana(ini, fin, existe_colision);
}

glm::vec3 ModeloMundo::calculaColisionMasCercana(glm::vec3 A, glm::vec3 B, bool* existe_colision) {
    *existe_colision = false;
    bool flag = false, curr_existe_colision = false;
    glm::vec3 colision_mas_cercana(0.0f), curr_colision(0.0f);

    glm::vec3 C, D;
    // Colision con borde del mundo
    for (int i = 2; i < num_vertices_borde[idx_mundo] + 2; i++) {
        C = glm::vec3((*vertices)[6 * (*relacion_vertices)[i]], (*vertices)[6 * (*relacion_vertices)[i] + 1], 0.0f);
        if (i + 1 < num_vertices_borde[idx_mundo] + 2) {
            D = glm::vec3((*vertices)[6 * (*relacion_vertices)[i + 1]], (*vertices)[6 * (*relacion_vertices)[i + 1] + 1], 0.0f);
        }
        else {
            D = glm::vec3((*vertices)[6 * (*relacion_vertices)[2]], (*vertices)[6 * (*relacion_vertices)[2] + 1], 0.0f);
        }

        curr_existe_colision = segmentoSegmentoInterseccion(A, B, C, D, &curr_colision);
        *existe_colision |= curr_existe_colision;
        if (curr_existe_colision) {
            if (!flag || norm(curr_colision - real_pos_robot) < norm(colision_mas_cercana - real_pos_robot)) {
                flag = true;
                colision_mas_cercana = curr_colision;
            }
        }
    }

    // Colision con objetos
    for (int i = num_vertices_borde[idx_mundo] + 2; i < num_indices[idx_mundo]; i += 3) {
        C = glm::vec3((*vertices)[6 * (*relacion_vertices)[i]], (*vertices)[6 * (*relacion_vertices)[i] + 1], 0.0f);
        D = glm::vec3((*vertices)[6 * (*relacion_vertices)[i + 1]], (*vertices)[6 * (*relacion_vertices)[i + 1] + 1], 0.0f);
        curr_existe_colision = segmentoSegmentoInterseccion(A, B, C, D, &curr_colision);
        
        *existe_colision |= curr_existe_colision;
        if (curr_existe_colision) {
            if (!flag || norm(curr_colision - real_pos_robot) < norm(colision_mas_cercana - real_pos_robot)) {
                flag = true;
                colision_mas_cercana = curr_colision;
            }
        }

        C = glm::vec3((*vertices)[6 * (*relacion_vertices)[i + 1]], (*vertices)[6 * (*relacion_vertices)[i + 1] + 1], 0.0f);
        D = glm::vec3((*vertices)[6 * (*relacion_vertices)[i + 2]], (*vertices)[6 * (*relacion_vertices)[i + 2] + 1], 0.0f);
        curr_existe_colision = segmentoSegmentoInterseccion(A, B, C, D, &curr_colision);

        *existe_colision |= curr_existe_colision;
        if (curr_existe_colision) {
            if (!flag || norm(curr_colision - real_pos_robot) < norm(colision_mas_cercana - real_pos_robot)) {
                flag = true;
                colision_mas_cercana = curr_colision;
            }
        }

        C = glm::vec3((*vertices)[6 * (*relacion_vertices)[i + 2]], (*vertices)[6 * (*relacion_vertices)[i + 2] + 1], 0.0f);
        D = glm::vec3((*vertices)[6 * (*relacion_vertices)[i]], (*vertices)[6 * (*relacion_vertices)[i] + 1], 0.0f);
        curr_existe_colision |= segmentoSegmentoInterseccion(A, B, C, D, &curr_colision);

        *existe_colision |= curr_existe_colision;
        if (curr_existe_colision) {
            if (!flag || norm(curr_colision - real_pos_robot) < norm(colision_mas_cercana - real_pos_robot)) {
                flag = true;
                colision_mas_cercana = curr_colision;
            }
        }
    }

    return colision_mas_cercana;
}


float ModeloMundo::getAngulo(glm::vec3 u, glm::vec3 v) {
    float theta;

    if (abs(u.x - v.x) < epsilon) {
        if (u.y < v.y) {
            theta = 90.0f;
        }
        else {
            theta = 270.0f;
        }
    }
    else if (abs(u.y - v.y) < epsilon) {
        if (u.x < v.x) {
            theta = 0.0f;
        }
        else {
            theta = 180.0f;
        }
    }
    else {
        theta = atan2(v.y - u.y, v.x - u.x) * 180 / M_PI;
    }

    return theta;
}

void ModeloMundo::calculaRVG() {
    cnt_vertices = 0;
    id_to_vertices.clear();
    rvg_aristas_pendientes.clear();
    rvg_vertices.clear();
    adj_vertices.clear();
    // Reflex vertex para la frontera del mundo
    // ordenados en sentido antihorario que den giro a la derecha
    int cnt_primer_vertice = -1;
    for(int i = 0; i < descripcion_mundo[idx_mundo][0]; i++) {
        int j = (i + 1) % descripcion_mundo[idx_mundo][0];
        int k = (i + 2) % descripcion_mundo[idx_mundo][0];

        glm::vec3 u((*vertices)[(i + 2) * 6], (*vertices)[(i + 2) * 6 + 1], 0.0f);
        glm::vec3 v((*vertices)[(j + 2) * 6], (*vertices)[(j + 2) * 6 + 1], 0.0f);
        glm::vec3 w((*vertices)[(k + 2) * 6], (*vertices)[(k + 2) * 6 + 1], 0.0f);

        int tipo_de_giro = orientacion(u, v, w);

        if (tipo_de_giro == 1) {
            // v es reflex vertex
            if (!i) cnt_primer_vertice = cnt_vertices;

            id_to_vertices[cnt_vertices] = v;
            relacion_rvg_vertices.push_back(cnt_vertices);

            rvg_vertices.push_back(v.x);
            rvg_vertices.push_back(v.y);
            rvg_vertices.push_back(0.59f);
            rvg_vertices.push_back(0.69f);
            rvg_vertices.push_back(1.00f);
            rvg_vertices.push_back(0.00f);

            // Si u fue reflex vertex hay que crear la arista
            if (cnt_vertices && id_to_vertices[cnt_vertices - 1] == u){
                rvg_aristas_pendientes.push_back({ cnt_vertices - 1, cnt_vertices });
                relacion_rvg_aristas.push_back(cnt_vertices - 1);
                relacion_rvg_aristas.push_back(cnt_vertices);
            }
            if (i == descripcion_mundo[idx_mundo][0] - 1 && cnt_primer_vertice != -1) {
                rvg_aristas_pendientes.push_back({ cnt_vertices, cnt_primer_vertice });
                relacion_rvg_aristas.push_back(cnt_vertices);
                relacion_rvg_aristas.push_back(cnt_primer_vertice);
            }

            adj_vertices.push_back({u, w});

            cnt_vertices++;
        }
    }


    // Reflex vertex para la frontera de hoyos
    // ordenados en sentido antihorario que den giro a la izquierda
    int vertices_acum = descripcion_mundo[idx_mundo][0];
    for (int id_obstaculo = 1; id_obstaculo < descripcion_mundo[idx_mundo].size(); id_obstaculo++) {
        cnt_primer_vertice = -1;
        for (int i = 0; i < descripcion_mundo[idx_mundo][id_obstaculo]; i++) {
            int j = (i + 1) % descripcion_mundo[idx_mundo][id_obstaculo];
            int k = (i + 2) % descripcion_mundo[idx_mundo][id_obstaculo];

            int real_i = i + vertices_acum;
            int real_j = j + vertices_acum;
            int real_k = k + vertices_acum;

            glm::vec3 u((*vertices)[(real_i + 2) * 6], (*vertices)[(real_i + 2) * 6 + 1], 0.0f);
            glm::vec3 v((*vertices)[(real_j + 2) * 6], (*vertices)[(real_j + 2) * 6 + 1], 0.0f);
            glm::vec3 w((*vertices)[(real_k + 2) * 6], (*vertices)[(real_k + 2) * 6 + 1], 0.0f);

            int tipo_de_giro = orientacion(u, v, w);

            if (tipo_de_giro == 2) {
                // v es reflex vertex
                if (!i) cnt_primer_vertice = cnt_vertices;

                id_to_vertices[cnt_vertices] = v;
                relacion_rvg_vertices.push_back(cnt_vertices);

                rvg_vertices.push_back(v.x);
                rvg_vertices.push_back(v.y);
                rvg_vertices.push_back(0.59f);
                rvg_vertices.push_back(0.69f);
                rvg_vertices.push_back(1.00f);
                rvg_vertices.push_back(0.00f);

                // Si u fue reflex vertex hay que crear la arista
                if (cnt_vertices && id_to_vertices[cnt_vertices - 1] == u) {
                    rvg_aristas_pendientes.push_back({ cnt_vertices - 1, cnt_vertices });
                    relacion_rvg_aristas.push_back(cnt_vertices - 1);
                    relacion_rvg_aristas.push_back(cnt_vertices);
                }
                if (i == descripcion_mundo[idx_mundo][id_obstaculo]-1 && cnt_primer_vertice != -1) {
                    rvg_aristas_pendientes.push_back({ cnt_vertices, cnt_primer_vertice });
                    relacion_rvg_aristas.push_back(cnt_vertices);
                    relacion_rvg_aristas.push_back(cnt_primer_vertice);
                }

                adj_vertices.push_back({ u, w });

                cnt_vertices++;
            }
        }
        vertices_acum += descripcion_mundo[idx_mundo][id_obstaculo];
    }

    // Aristas de vertices adyacentes
    rvg.resize(rvg_vertices.size());
    for (auto e : rvg_aristas_pendientes) {
        rvg[e.first].push_back(e.second);
        rvg[e.second].push_back(e.first);
    }

    // Aristas de bitangentes
    for (int i = 0; i + 1 < cnt_vertices; i++) {
        glm::vec3 u = id_to_vertices[i], v, ini, fin, d;
        glm::vec3 p1, p2, p3, p4, p5, p6;

        p1 = adj_vertices[i].first;
        p2 = u;
        p3 = adj_vertices[i].second;

        for (int j = i + 1; j < cnt_vertices; j++) {
            v = id_to_vertices[j];

            bool existe_colision = false;
            calculaColisionMasCercanaSinExtremos(u, v, &existe_colision);

            if (existe_colision) continue;
            
            p4 = adj_vertices[j].first;
            p5 = v;
            p6 = adj_vertices[j].second;

            int test = (orientacionBinariaNoColineal(p1, p2, p5)^orientacionBinariaNoColineal(p3, p2, p5))|(orientacionBinariaNoColineal(p4, p5, p2)^orientacionBinariaNoColineal(p6, p5, p2));

            if (!test) {
                relacion_rvg_aristas.push_back(i);
                relacion_rvg_aristas.push_back(j);

                rvg[i].push_back(j);
                rvg[j].push_back(i);
            }
        }
    }
    return;
}