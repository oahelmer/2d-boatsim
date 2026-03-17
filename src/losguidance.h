#ifndef LOS_GUIDANCE_H
#define LOS_GUIDANCE_H

#include "boat.h"
#include "constants.h"
#include <vector>

class LOSController {
public:
    LOSController(const std::vector<Waypoint>& path);
    
    // Returns {F, tau}
    std::pair<float, float> computeControl(const State& s, float dt);

private:
    std::vector<Waypoint> waypoints;
    int currentTargetIdx = 1;
    float delta = DELTA; // Look-ahead distance
    
    // PID/PD internal states for heading
    float prevThetaError = 0;
};

#endif