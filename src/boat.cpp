#include "boat.h"
#include "constants.h"
#include <GL/glew.h>
#include <cmath>

Boat::Boat() {
    // Initialize all 8 state variables: x, vx, y, vy, theta, omega, phi, dphi
    state = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
}

State Boat::f(State s, float F, float tau) {
    return {
        s.vx, 
        (F * sinf(s.theta + s.phi) - DAMPING_V * s.vx) / M,
        s.vy, 
        (F * cosf(s.theta + s.phi) - DAMPING_V * s.vy) / M,
        s.omega, 
        (L * F * sinf(s.phi) - DAMPING_W * s.omega) / I,
        s.dphi,
        (tau - DAMPING_PHI * s.dphi) / I_MOTOR
    };
}

void Boat::update(float dt, float F, float tau) {
    // Helper lambda to add states (useful for RK4 steps)
    auto add = [](State s, State k, float factor) -> State {
        return {
            s.x + k.x * factor, s.vx + k.vx * factor,
            s.y + k.y * factor, s.vy + k.vy * factor,
            s.theta + k.theta * factor, s.omega + k.omega * factor,
            s.phi + k.phi * factor, s.dphi + k.dphi * factor
        };
    };

    // RK4 Integration Step
    State k1 = f(state, F, tau);
    State k2 = f(add(state, k1, dt/2.0f), F, tau);
    State k3 = f(add(state, k2, dt/2.0f), F, tau);
    State k4 = f(add(state, k3, dt), F, tau);

    // Final state update across all 8 variables
    state.x     += (dt/6.0f) * (k1.x + 2*k2.x + 2*k3.x + k4.x);
    state.vx    += (dt/6.0f) * (k1.vx + 2*k2.vx + 2*k3.vx + k4.vx);
    state.y     += (dt/6.0f) * (k1.y + 2*k2.y + 2*k3.y + k4.y);
    state.vy    += (dt/6.0f) * (k1.vy + 2*k2.vy + 2*k3.vy + k4.vy);
    state.theta += (dt/6.0f) * (k1.theta + 2*k2.theta + 2*k3.theta + k4.theta);
    state.omega += (dt/6.0f) * (k1.omega + 2*k2.omega + 2*k3.omega + k4.omega);
    state.phi   += (dt/6.0f) * (k1.phi + 2*k2.phi + 2*k3.phi + k4.phi);
    state.dphi  += (dt/6.0f) * (k1.dphi + 2*k2.dphi + 2*k3.dphi + k4.dphi);
}

void Boat::draw(float r, float g, float b) {
    glPushMatrix();
    glTranslatef(state.x, state.y, 0);
    glRotatef(-state.theta * 180.0f / 3.14159f, 0, 0, 1);
    
    // Draw Boat Hull
    glBegin(GL_TRIANGLES);
    glColor3f(r,g,b);
    glVertex2f(0.0f, 0.5f + L);   // front of boat
    glVertex2f(-0.3f, -0.5f - L); // back left of boat
    glVertex2f(0.3f, -0.5f - L);  // back right of boat
    glEnd();

    // Draw Motor (Red Line)
    glPushMatrix();
    glTranslatef(0.0f, -0.5f - L, 0.0f); // Move to rear center of boat
    glRotatef(state.phi * 180.0f / 3.14159f, 0, 0, 1); // Rotate relative to boat hull
    
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f); // Red
    glVertex2f(0.0f, 0.0f);
    glVertex2f(0.0f, -0.6f);    // Points backwards from the pivot
    glEnd();
    glPopMatrix();

    glPopMatrix();
}