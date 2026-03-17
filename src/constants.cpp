#include "constants.h"

const std::vector<Waypoint> GLOBAL_PATH = {{0,0}, {50,0}, {50,50}, {0,50}, {0,0}};

const float M = 5.0f;
const float L = 1.0f;
const float I = M*L*L * (1.0f/12.0f);
const float F_MAX = 5.0f;
const float T_MAX = 2.0f;
const float DAMPING_V = 1.8f; 
const float DAMPING_W = 1.2f*L;
const float I_MOTOR = 0.05f; 
const float DAMPING_PHI = 1.5f;

// --- LOS & Control Tuning ---
const float DELTA = 6.0f;      // Fossen's Delta (look-ahead). 6.0 is stable for this boat size.
const float WP_RADIUS = 3.0f;  // Radius for changing waypoints
const float K_HEADING = 0.5f;  // P-gain for heading
const float K_PHI_P = 15.0f;   // Proportional motor stiffness
const float K_PHI_D = 5.0f;    // Derivative motor damping (CRITICAL for oscillations)
const float SPEED_FAC = 0.6f;