#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "boat.h"
#include "constants.h"

float currentF = 0, currentTau = 0;

void processInput(GLFWwindow* window) {
    currentF = 0; currentTau = 0;
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) currentF = F_MAX;
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) currentF = -F_MAX;
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) currentTau = -T_MAX;
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) currentTau = T_MAX;
}

void drawReferenceSquares() {
    float positions[4][2] = {{2,2}, {-2,2}, {-2,-2}, {2,-2}};
    glColor3f(0.5f, 0.5f, 0.5f);
    for(int i=0; i<4; i++) {
        glBegin(GL_QUADS);
        glVertex2f(positions[i][0]-0.2f, positions[i][1]-0.2f);
        glVertex2f(positions[i][0]+0.2f, positions[i][1]-0.2f);
        glVertex2f(positions[i][0]+0.2f, positions[i][1]+0.2f);
        glVertex2f(positions[i][0]-0.2f, positions[i][1]+0.2f);
        glEnd();
    }
}

int main() {
    if (!glfwInit()) return -1;
    GLFWwindow* window = glfwCreateWindow(1800, 1800, "Boat Sim", NULL, NULL);
    glfwMakeContextCurrent(window);
    glewInit();

    Boat boat;
    float lastTime = glfwGetTime();

    while (!glfwWindowShouldClose(window)) {
        float currentTime = glfwGetTime();
        float dt = currentTime - lastTime;
        lastTime = currentTime;

        processInput(window);
        boat.update(dt, currentF, currentTau);

        glClear(GL_COLOR_BUFFER_BIT);
        glLoadIdentity();

        // Camera following logic
        glScalef(0.1f, 0.1f, 1.0f); // Zoom out a bit
        glTranslatef(-boat.state.x, -boat.state.y, 0);

        drawReferenceSquares();
        boat.draw();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glfwTerminate();
    return 0;
}