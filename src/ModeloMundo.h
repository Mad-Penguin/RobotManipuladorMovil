#pragma once
#include <glm.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <gtc/type_ptr.hpp>
#include <algorithm>
#include <math.h>
#include <vector>
#include <map>
#include <queue>

#define M_PI 3.14159265358979323846f

enum class MUNDOS{
    MUNDO_1 = 0,
    MUNDO_2 = 1,
    MUNDO_3 = 2,
    MUNDO_4 = 3,
};

class ModeloMundo {
private:
    // Variables que describiran el mundo
    MUNDOS tipo_de_mundo;
    std::vector<float>* vertices;
    std::vector<unsigned int>* relacion_vertices;
    int idx_mundo = 0;

    float epsilon = 1e-5, d_theta = 0.1f;
    float distance_to_leave_world = 10.0f;

    // Sobre el RVG
    int cnt_vertices = 0;
    float muestra_rvg, muestra_camino;
    std::map<int, glm::vec3> id_to_vertices;
    std::vector<std::pair<glm::vec3, glm::vec3>> adj_vertices;
    std::vector<std::vector<int>> rvg;
    std::vector<std::pair<int, int>> rvg_aristas_pendientes;

    // RVG en OpenGL
    std::vector<float> rvg_vertices;
    std::vector<unsigned int> relacion_rvg_aristas, relacion_rvg_vertices;

    // Camino en OpenGL
    bool need_update = true;
    std::vector<float> camino_vertices;
    std::vector<unsigned int> relacion_camino_aristas;

    // Variables sobre el algoritmo
    glm::vec3 pos_robot, movimiento_acumulado_robot;
    glm::vec3 real_pos_robot;

    // Variables relacionadas a OpenGL
    int height, width;
    unsigned int shader, buffer, index_buffer;
    unsigned int buffer_rvg, index_buffer_rvg_aristas, index_buffer_rvg_vertices;
    unsigned int buffer_camino, index_buffer_camino;
    int model_location, view_location, proj_location;

    bool enSegmento(glm::vec3 p, glm::vec3 q, glm::vec3 r);
    int orientacion(glm::vec3 p, glm::vec3 q, glm::vec3 r);
    int orientacionBinariaNoColineal(glm::vec3 p, glm::vec3 q, glm::vec3 r);
    bool segmentoSegmentoInterseccion(glm::vec3 A, glm::vec3 B, glm::vec3 C, glm::vec3 D, glm::vec3 *colision_mas_cercana = nullptr);
    float norm(glm::vec3 v);

    float getAngulo(glm::vec3 u, glm::vec3 v);
    glm::vec3 calculaColisionMasCercana(glm::vec3 A, glm::vec3 B, bool *existe_colision);
    glm::vec3 calculaColisionMasCercanaSinExtremos(glm::vec3 A, glm::vec3 B, bool *existe_colision);

    void calculaRVG();
    void dibujaRVG();

    void genCamino();
    void dibujaCamino();

public:
    ModeloMundo(MUNDOS _tipo_de_mundo_, int _height_, int _width_, unsigned int _shader_);
    void dibujaMundo();

    void asignaPuntoInicial(float x, float y);
    void asignaPuntoFinal(float x, float y);
    void muestraRVG(bool flag);
    void muestraCamino(bool flag);

private:
    std::vector<std::vector<int>> descripcion_mundo = {
        // Cantidad de vertices del mundo y luego por cada hoyo
        {12, 4},
        {15, 3},
        {4, 3, 3, 3, 3, 3, 3, 3, 3},
        {30, 31}
    };

    std::vector<float> mundo_1_vertices = {
        // Coordenadas      // RGBA
        // Robot
         0.0f, -1.75f,        0.37f, 0.74f, 0.3f, 0.0f,
         0.0f,  1.75f,        1.00f, 0.93f, 0.0f, 0.0f,

         // Bordes mundo
         -3.4f,-1.9f,         1.0f, 0.15f, 0.25f, 0.0f,
          3.4,-1.9f,         1.0f, 0.15f, 0.25f, 0.0f,
          0.5f,-1.0f,      1.0f, 0.15f, 0.25f, 0.0f,
          0.2f,-1.0f,      1.0f, 0.15f, 0.25f, 0.0f,
          0.2f,1.0f,      1.0f, 0.15f, 0.25f, 0.0f,
          0.5f,1.0f,   1.0f, 0.15f, 0.25f, 0.0f,
          3.4f, 1.9f,  1.0f, 0.15f, 0.25f, 0.0f,
          -3.4, 1.9f,      1.0f, 0.15f, 0.25f, 0.0f,
          -0.5f,1.0f,     1.0f, 0.15f, 0.25f, 0.0f,
          -0.2f, 1.0f,        1.0f, 0.15f, 0.25f, 0.0f,
          -0.2f, -1.0f,        1.0f, 0.15f, 0.25f, 0.0f,
          -0.5f, -1.0f,       1.0f, 0.15f, 0.25f, 0.0f,

          // Rectangulo
          -1.0f, -1.6f, 1.0f, 0.15f, 0.25f, 0.0f,
           1.0f, -1.6f, 1.0f, 0.15f, 0.25f, 0.0f,
           1.0f, -1.4f, 1.0f, 0.15f, 0.25f, 0.0f,
          -1.0f, -1.4f, 1.0f, 0.15f, 0.25f, 0.0f,
    };

    std::vector<unsigned int> mundo_1_relaciones = {
        0,
        1,
        2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,



        14,15,16,
        16,17,14
    };

    std::vector<float> mundo_2_vertices = {
        // Coordenadas      // RGBA
        // Robot
         0.9390f, -1.7f, 0.37f, 0.74f, 0.3f, 0.0f,
         0.9390f,  1.7f, 1.00f, 0.93f, 0.0f, 0.0f,

        
        // Bordes mundo
        -2.6038f,1.388f,    1.0f, 0.15f, 0.25f, 0.0f,
        -2.0f, 0.0f,        1.0f, 0.15f, 0.25f, 0.0f,
        -2.6124f,-0.8840f,  1.0f, 0.15f, 0.25f, 0.0f,
        -1.5f,-0.5f,        1.0f, 0.15f, 0.25f, 0.0f,
        1.5f,-0.5f,        1.0f, 0.15f, 0.25f, 0.0f,
        1.65f,-1.3f,        1.0f, 0.15f, 0.25f, 0.0f,
        0.5f,-1.2987f,     1.0f, 0.15f, 0.25f, 0.0f,
        0.65f,-1.799f,      1.0f, 0.15f, 0.25f, 0.0f,
        2.7529f,-1.7912f,  1.0f, 0.15f, 0.25f, 0.0f,
        2.7529f,1.7856f,   1.0f, 0.15f, 0.25f, 0.0f,
        0.65f,1.7856f,      1.0f, 0.15f, 0.25f, 0.0f,
        0.5f,1.2845f,      1.0f, 0.15f, 0.25f, 0.0f,
        1.65f,1.2845f,      1.0f, 0.15f, 0.25f, 0.0f,
        1.5f,0.5f,         1.0f, 0.15f, 0.25f, 0.0f,
        -1.5f,0.5f,         1.0f, 0.15f, 0.25f, 0.0f,

        // Triangulo
         0.2, -0.2,         1.0f, 0.15f, 0.25f, 0.0f,
         1.2, -0.2,         1.0f, 0.15f, 0.25f, 0.0f,
         0.7,  0.2,         1.0f, 0.15f, 0.25f, 0.0f,
    };

    std::vector<unsigned int> mundo_2_relaciones = {
        0,
        1,
        2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
        17, 18, 19,
    };

    std::vector<float> mundo_3_vertices = {
        // Coordenadas      // RGBA
        // Robot
        0.0f, 1.8f,  0.37f, 0.74f, 0.3f, 0.0f,
        0.0f,-1.8f,  1.00f, 0.93f, 0.0f, 0.0f,

        // Bordes del mundo
       -2.5f, -1.9f,  1.0f, 0.15f, 0.25f, 0.0f,
        2.5f, -1.9f, 1.0f, 0.15f, 0.25f, 0.0f,
        2.5f,  1.9f, 1.0f, 0.15f, 0.25f, 0.0f,
       -2.5f,  1.9f, 1.0f, 0.15f, 0.25f, 0.0f,

        // Obstaculos
        0.0f,-0.1f,  1.0f, 0.15f, 0.25f, 0.0f,
       -0.375f,-0.7f,  1.0f, 0.15f, 0.25f, 0.0f,
        0.375f,-0.7f, 1.0f, 0.15f, 0.25f, 0.0f,
        
        0.1f, 0.0f,  1.0f, 0.15f, 0.25f, 0.0f,
        0.7f, -0.375f, 1.0f, 0.15f, 0.25f, 0.0f,
        0.7f, 0.375f, 1.0f, 0.15f, 0.25f, 0.0f,

        0.0f, 0.1f, 1.0f, 0.15f, 0.25f, 0.0f,
        0.375f, 0.7f, 1.0f, 0.15f, 0.25f, 0.0f,
       -0.375f, 0.7f, 1.0f, 0.15f, 0.25f, 0.0f,
        
       -0.1f, 0.0f, 1.0f, 0.15f, 0.25f, 0.0f,
       -0.7f, 0.375f, 1.0f, 0.15f, 0.25f, 0.0f,
       -0.7f, -0.375f, 1.0f, 0.15f, 0.25f, 0.0f,

       0.0f,1.5f,  1.0f, 0.15f, 0.25f, 0.0f,
       -0.375f,0.9f,  1.0f, 0.15f, 0.25f, 0.0f,
        0.375f,0.9f, 1.0f, 0.15f, 0.25f, 0.0f,
            
        1.5f, 0.0f, 1.0f, 0.15f, 0.25f, 0.0f,
        0.9f, 0.375f, 1.0f, 0.15f, 0.25f, 0.0f,
        0.9f, -0.375f, 1.0f, 0.15f, 0.25f, 0.0f,

        0.0f,-1.5f,  1.0f, 0.15f, 0.25f, 0.0f,
        0.375f,-0.9f, 1.0f, 0.15f, 0.25f, 0.0f,
       -0.375f,-0.9f,  1.0f, 0.15f, 0.25f, 0.0f,

        -1.5f, 0.0f,  1.0f, 0.15f, 0.25f, 0.0f,
        -0.9f, -0.375f, 1.0f, 0.15f, 0.25f, 0.0f,
        -0.9f, 0.375f, 1.0f, 0.15f, 0.25f, 0.0f,
    };

    std::vector<unsigned int> mundo_3_relaciones = {
        0,
        1,
        2,3,4,5,

        6,7,8,
        9,10,11,
        12,13,14,
        15,16,17,

        18,19,20,
        21,22,23,
        24,25,26,
        27,28,29
    };

    std::vector<float> mundo_4_vertices = {
        // Coordenadas      // RGBA
        // Robot
       -1.175f, 0.0f,  0.37f, 0.74f, 0.3f, 0.0f,
        1.175f, 0.0f,  1.00f, 0.93f, 0.0f, 0.0f,

        // Bordes del mundo
        1.8500, 0.0000, 1.0f, 0.15f, 0.25f, 0.0f,
1.8096, 0.3846, 1.0f, 0.15f, 0.25f, 0.0f,
1.6901, 0.7525, 1.0f, 0.15f, 0.25f, 0.0f,
1.4967, 1.0874, 1.0f, 0.15f, 0.25f, 0.0f,
1.2379, 1.3748, 1.0f, 0.15f, 0.25f, 0.0f,
0.9250, 1.6021, 1.0f, 0.15f, 0.25f, 0.0f,
0.5717, 1.7595, 1.0f, 0.15f, 0.25f, 0.0f,
0.1934, 1.8399, 1.0f, 0.15f, 0.25f, 0.0f,
-0.1934, 1.8399, 1.0f, 0.15f, 0.25f, 0.0f,
-0.5717, 1.7595, 1.0f, 0.15f, 0.25f, 0.0f,
-0.9250, 1.6021, 1.0f, 0.15f, 0.25f, 0.0f,
-1.2379, 1.3748, 1.0f, 0.15f, 0.25f, 0.0f,
-1.4967, 1.0874, 1.0f, 0.15f, 0.25f, 0.0f,
-1.6901, 0.7525, 1.0f, 0.15f, 0.25f, 0.0f,
-1.8096, 0.3846, 1.0f, 0.15f, 0.25f, 0.0f,
-1.8500, -0.0000, 1.0f, 0.15f, 0.25f, 0.0f,
-1.8096, -0.3846, 1.0f, 0.15f, 0.25f, 0.0f,
-1.6901, -0.7525, 1.0f, 0.15f, 0.25f, 0.0f,
-1.4967, -1.0874, 1.0f, 0.15f, 0.25f, 0.0f,
-1.2379, -1.3748, 1.0f, 0.15f, 0.25f, 0.0f,
-0.9250, -1.6021, 1.0f, 0.15f, 0.25f, 0.0f,
-0.5717, -1.7595, 1.0f, 0.15f, 0.25f, 0.0f,
-0.1934, -1.8399, 1.0f, 0.15f, 0.25f, 0.0f,
0.1934, -1.8399, 1.0f, 0.15f, 0.25f, 0.0f,
0.5717, -1.7595, 1.0f, 0.15f, 0.25f, 0.0f,
0.9250, -1.6021, 1.0f, 0.15f, 0.25f, 0.0f,
1.2379, -1.3748, 1.0f, 0.15f, 0.25f, 0.0f,
1.4967, -1.0874, 1.0f, 0.15f, 0.25f, 0.0f,
1.6901, -0.7525, 1.0f, 0.15f, 0.25f, 0.0f,
1.8096, -0.3846, 1.0f, 0.15f, 0.25f, 0.0f,

    /// circulo central
    0.0f, 0.0f, 1.0f, 0.15f, 0.25f, 0.0f,
0.5000, 0.0000, 1.0f, 0.15f, 0.25f, 0.0f,
0.4891, 0.1040, 1.0f, 0.15f, 0.25f, 0.0f,
0.4568, 0.2034, 1.0f, 0.15f, 0.25f, 0.0f,
0.4045, 0.2939, 1.0f, 0.15f, 0.25f, 0.0f,
0.3346, 0.3716, 1.0f, 0.15f, 0.25f, 0.0f,
0.2500, 0.4330, 1.0f, 0.15f, 0.25f, 0.0f,
0.1545, 0.4755, 1.0f, 0.15f, 0.25f, 0.0f,
0.0523, 0.4973, 1.0f, 0.15f, 0.25f, 0.0f,
-0.0523, 0.4973, 1.0f, 0.15f, 0.25f, 0.0f,
-0.1545, 0.4755, 1.0f, 0.15f, 0.25f, 0.0f,
-0.2500, 0.4330, 1.0f, 0.15f, 0.25f, 0.0f,
-0.3346, 0.3716, 1.0f, 0.15f, 0.25f, 0.0f,
-0.4045, 0.2939, 1.0f, 0.15f, 0.25f, 0.0f,
-0.4568, 0.2034, 1.0f, 0.15f, 0.25f, 0.0f,
-0.4891, 0.1040, 1.0f, 0.15f, 0.25f, 0.0f,
-0.5000, -0.0000, 1.0f, 0.15f, 0.25f, 0.0f,
-0.4891, -0.1040, 1.0f, 0.15f, 0.25f, 0.0f,
-0.4568, -0.2034, 1.0f, 0.15f, 0.25f, 0.0f,
-0.4045, -0.2939, 1.0f, 0.15f, 0.25f, 0.0f,
-0.3346, -0.3716, 1.0f, 0.15f, 0.25f, 0.0f,
-0.2500, -0.4330, 1.0f, 0.15f, 0.25f, 0.0f,
-0.1545, -0.4755, 1.0f, 0.15f, 0.25f, 0.0f,
-0.0523, -0.4973, 1.0f, 0.15f, 0.25f, 0.0f,
0.0523, -0.4973, 1.0f, 0.15f, 0.25f, 0.0f,
0.1545, -0.4755, 1.0f, 0.15f, 0.25f, 0.0f,
0.2500, -0.4330, 1.0f, 0.15f, 0.25f, 0.0f,
0.3346, -0.3716, 1.0f, 0.15f, 0.25f, 0.0f,
0.4045, -0.2939, 1.0f, 0.15f, 0.25f, 0.0f,
0.4568, -0.2034, 1.0f, 0.15f, 0.25f, 0.0f,
0.4891, -0.1040, 1.0f, 0.15f, 0.25f, 0.0f,
    };

    std::vector<unsigned int> mundo_4_relaciones = {
        0,
        1,
        2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,

        32, 33,34,
32, 34,35,
32, 35,36,
32, 36,37,
32, 37,38,
32, 38,39,
32, 39,40,
32, 40,41,
32, 41,42,
32, 42,43,
32, 43,44,
32, 44,45,
32, 45,46,
32, 46,47,
32, 47,48,
32, 48,49,
32, 49,50,
32, 50,51,
32, 51,52,
32, 52,53,
32, 53,54,
32, 54,55,
32, 55,56,
32, 56,57,
32, 57,58,
32, 58,59,
32, 59,60,
32, 60,61,
32, 61,62,
32, 62, 34,
    };

    std::vector<int> num_vertices = 
     { 
        (int)(mundo_1_vertices.size()) / 6,
        (int)(mundo_2_vertices.size()) / 6,
        (int)(mundo_3_vertices.size()) / 6,
        (int)(mundo_4_vertices.size()) / 6,
     };
    std::vector<int> num_indices =
    {
        (int)(mundo_1_relaciones.size()),
        (int)(mundo_2_relaciones.size()),
        (int)(mundo_3_relaciones.size()),
        (int)(mundo_4_relaciones.size()),
    };
    std::vector<int> num_vertices_borde =
    {
        12,
        15,
        4,
        30,
    };
};