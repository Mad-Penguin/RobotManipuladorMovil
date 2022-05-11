#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "ModeloMundo.h"
#include "ModeloRobot.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm.hpp>
#include <gtc/type_ptr.hpp>

#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_glfw.h>



struct ShaderProgramSource {
    std::string VertexSource;
    std::string FragmentSource;
};

static ShaderProgramSource ParseShader(const std::string& filepath) {
    std::ifstream stream(filepath);

    enum class ShaderType {
        NONE = -1,
        VERTEX = 0,
        FRAGMENT = 1
    };

    std::string line;
    std::stringstream ss[2];
    ShaderType type = ShaderType::NONE;

    while (std::getline(stream, line)) {
        if (line.find("#shader") != std::string::npos) {
            if (line.find("vertex") != std::string::npos)
                type = ShaderType::VERTEX;
            else if (line.find("fragment") != std::string::npos)
                type = ShaderType::FRAGMENT;
        }
        else {
            if (type != ShaderType::NONE)
                ss[(int)type] << line << "\n";
        }
    }

    return { ss[0].str(), ss[1].str() };
}

static unsigned int CompileShader(unsigned int type, const std::string& source) {
    unsigned int id = glCreateShader(type);
    const char* src = source.c_str();
    glShaderSource(id, 1, &src, nullptr);
    glCompileShader(id);

    int result;
    glGetShaderiv(id, GL_COMPILE_STATUS, &result);
    if (result == GL_FALSE) {
        int length;
        glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
        char* message = (char*)malloc(length * sizeof(char));
        glGetShaderInfoLog(id, length, &length, message);

        std::cout << "Error al compilar "
            << (type == GL_VERTEX_SHADER ? "vertex" : "fragment")
            << " shader." << "\n";
        std::cout << message << "\n";
        glDeleteShader(id);
        return 0;
    }

    return id;
}

static unsigned int createShader(const std::string& vertexShader, const std::string& fragmentShader) {
    unsigned int program = glCreateProgram();
    unsigned int vs = CompileShader(GL_VERTEX_SHADER, vertexShader);
    unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, fragmentShader);

    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glValidateProgram(program);

    glDeleteShader(vs);
    glDeleteShader(fs);

    return program;
}

int main(void)
{
    GLFWwindow* window;

    /* Initialize the library */
    if (!glfwInit())
        return -1;

    int height = 1280, width = 720;
    //int height = 960, width = 720;
    //int height = 640, width = 480;    // The Cherno
    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(height, width, "Robot Manipulador Movil - DenavitHartenberg", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);

    glfwSwapInterval(5);

    if (glewInit() != GLEW_OK)
        std::cout << "Error con glewInit()" << "\n";

    std::cout << glGetString(GL_VERSION) << "\n";

    ShaderProgramSource source = ParseShader("res/shaders/Basic.shader");
    unsigned int shader = createShader(source.VertexSource, source.FragmentSource);

    glUseProgram(shader);
    glEnable(GL_DEPTH_TEST);

    int location = glGetUniformLocation(shader, "u_Color");

    //Setup IMGUI
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init((char*)glGetString(GL_NUM_SHADING_LANGUAGE_VERSIONS));

    glm::vec2 xy(0.0f);
    float rotacion_base = 0.0f;

    bool muestra_ejes = false;

    // DELETE
    int parts = 9;
    glm::vec3 traslada_frame(0.0f);

    ModeloRobot robot("E:/TareasProyectos/8vo/Robotica/RobotManipuladorMovil-DenavitHartenberg/Robot/OBJ_Robot.obj", height, width, shader, true);

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {
        /* Render here */
        //New Frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        //glEnable(GL_BLEND);
        //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glUseProgram(shader);
        robot.how_many_parts = parts;
        
        robot.moveLastFrame(traslada_frame);

        robot.rotaBase(rotacion_base);
        robot.traslada(glm::vec3(xy,0.0f));
        robot.muestraEjes(muestra_ejes);

        robot.dibujaRobot();

        {
            static float f = 0.0f;
            static int counter = 0;

            ImGui::Begin("UI");

            ImGui::Checkbox("Muestra ejes",&muestra_ejes);
            ImGui::SliderFloat3("Traslada frame", &traslada_frame.x, -15.0f, 15.0f);

            ImGui::SliderFloat3("Traslacion", &xy.x, -15.0f, 15.0f);
            ImGui::SliderFloat("Rotacion", &rotacion_base, 0.0f, 360.0f);
            ImGui::SliderInt("Partes", &parts, 0, 9);

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
            ImGui::End();
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glDeleteProgram(shader);
    glfwTerminate();

    return 0;
}