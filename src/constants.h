#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <vector>

struct Waypoint {
    float x, y;
};

extern const std::vector<Waypoint> GLOBAL_PATH;

extern const float M;
extern const float L;
extern const float I;
extern const float F_MAX;
extern const float T_MAX;
extern const float DAMPING_V;
extern const float DAMPING_W;
extern const float I_MOTOR;
extern const float DAMPING_PHI;

// --- LOS & Control Gains ---
extern const float DELTA;      
extern const float WP_RADIUS;  
extern const float K_HEADING;  
extern const float K_PHI_P;    
extern const float K_PHI_D;    
extern const float SPEED_FAC;  

#endif