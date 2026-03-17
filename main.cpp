#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "boat.h"
#include "constants.h"
#include "losguidance.h"

float currentF = 0, currentTau = 0;

void processInput(GLFWwindow* window) {
    currentF = 0; currentTau = 0;
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) currentF = F_MAX;
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) currentF = -F_MAX;
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) currentTau = -T_MAX;
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) currentTau = T_MAX;
}

void drawReferenceSquares() {
    float positions[4][2] =  {{2,2}, {-2,2}, {-2,-2}, {2,-2}};
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

void drawPath() {
    // Draw the lines connecting waypoints
    glLineWidth(2.0f);
    glColor3f(0.4f, 0.4f, 0.4f);
    glBegin(GL_LINE_STRIP);
    for(const auto& wp : GLOBAL_PATH) {
        glVertex2f(wp.x, wp.y);
    }
    glEnd();

    // Draw square markers at waypoints
    glColor3f(0.5f, 0.5f, 0.5f);
    for(const auto& wp : GLOBAL_PATH) {
        glBegin(GL_QUADS);
        float s = 0.5f;
        glVertex2f(wp.x - s, wp.y - s);
        glVertex2f(wp.x + s, wp.y - s);
        glVertex2f(wp.x + s, wp.y + s);
        glVertex2f(wp.x - s, wp.y + s);
        glEnd();
    }
}

int main() {
    if (!glfwInit()) return -1;
    GLFWwindow* window = glfwCreateWindow(1800, 1800, "Boat Sim", NULL, NULL);
    glfwMakeContextCurrent(window);
    glewInit();

    Boat boat;
    Boat playerBoat;
    Boat guidedBoat;

    // Define a square path
    std::vector<Waypoint> path = {{0,0}, {50,0}, {50,50}, {0,50}, {0,0}};
    LOSController controller(path);

    float lastTime = glfwGetTime();

    while (!glfwWindowShouldClose(window)) {
        float currentTime = glfwGetTime();
        float dt = currentTime - lastTime;
        lastTime = currentTime;

        // Player Control
        processInput(window);
        playerBoat.update(dt, currentF, currentTau);

        // AI Control
        auto [aiF, aiTau] = controller.computeControl(guidedBoat.state, dt);
        guidedBoat.update(dt, aiF, aiTau);

        glClear(GL_COLOR_BUFFER_BIT);
        glLoadIdentity();

        // Camera following logic
        glScalef(0.05f, 0.05f, 1.0f); // Zoom out a bit
        glTranslatef(-playerBoat.state.x, -playerBoat.state.y, 0);

        drawReferenceSquares();
        drawPath();
        playerBoat.draw();
        guidedBoat.draw();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glfwTerminate();
    return 0;
}