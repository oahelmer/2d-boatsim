#include "losguidance.h"
#include "constants.h"
#include <cmath>
#include <algorithm>

LOSController::LOSController(const std::vector<Waypoint>& path) : waypoints(path) {}

std::pair<float, float> LOSController::computeControl(const State& s, float dt) {
    if (waypoints.size() < 2) return {0, 0};

    Waypoint p0 = waypoints[currentTargetIdx - 1];
    Waypoint p1 = waypoints[currentTargetIdx];

    // 1. Path Angle (gamma_p) - 0 is Up, pi/2 is Right
    float dx = p1.x - p0.x;
    float dy = p1.y - p0.y;
    float gamma_p = atan2f(dx, dy); 

    // 2. Correct Cross-track error (e) for CW system
    // We project the position error onto the path's normal vector (Starboard)
    float errX = s.x - p0.x;
    float errY = s.y - p0.y;
    float e = errX * cosf(gamma_p) - errY * sinf(gamma_p);

    // 3. LOS Guidance Law (Fossen)
    float theta_d = gamma_p + atanf(-e / DELTA);

    // 4. Heading Control
    float angleDiff = theta_d - s.theta;
    while (angleDiff > M_PI)  angleDiff -= 2.0f * M_PI;
    while (angleDiff < -M_PI) angleDiff += 2.0f * M_PI;

    float target_phi = angleDiff * K_HEADING; 
    target_phi = std::clamp(target_phi, -0.7f, 0.7f);

    // PD Control for Motor
    float phi_error = target_phi - s.phi;
    float tau = (phi_error * K_PHI_P) - (s.dphi * K_PHI_D);
    tau = std::clamp(tau, -T_MAX, T_MAX);

    // 5. Waypoint Switching
    float distSq = dx*dx + dy*dy; // Segment length squared
    float progress = (errX * dx + errY * dy) / distSq; // Normalized progress along segment
    
    // Switch if we are close to p1 or have passed it
    float distToP1 = sqrtf(powf(p1.x - s.x, 2) + powf(p1.y - s.y, 2));
    if (distToP1 < WP_RADIUS && currentTargetIdx < (int)waypoints.size() - 1) {
        currentTargetIdx++;
    }

    return {F_MAX * SPEED_FAC, tau};
}