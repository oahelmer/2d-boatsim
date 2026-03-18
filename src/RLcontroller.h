#ifndef RL_CONTROLLER_H
#define RL_CONTROLLER_H

#include "boat.h"
#include "constants.h"
#include <vector>
#include <string>
#include <onnxruntime_cxx_api.h> // Requires ONNX Runtime

class RLController {
public:
    // Constructor now takes the path to the .onnx model file
    RLController(const std::vector<Waypoint>& path, const std::string& modelPath);
    
    std::pair<float, float> computeControl(const State& s, float dt);

private:
    std::vector<Waypoint> waypoints;
    int currentTargetIdx = 1;

    // ONNX Runtime components
    Ort::Env env;
    Ort::Session session;
    Ort::MemoryInfo memoryInfo;

    // Model metadata
    const char* inputName = "input";
    const char* outputName = "output";

    // Internal inference method
    std::vector<float> runInference(const std::vector<float>& observation);
};

#endif