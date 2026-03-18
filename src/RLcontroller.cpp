#include "RLcontroller.h"
#include <cmath>
#include <algorithm>
#include <iostream>

// Constructor initializes the ONNX session
RLController::RLController(const std::vector<Waypoint>& path, const std::string& modelPath) 
    : waypoints(path), 
      env(ORT_LOGGING_LEVEL_WARNING, "RLController"),
      session(env, modelPath.c_str(), Ort::SessionOptions{nullptr}),
      memoryInfo(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) 
{}

std::vector<float> RLController::runInference(const std::vector<float>& observation) {
    // 1. Define the shape of our input tensor (1 instance, 7 features)
    std::vector<int64_t> inputShape = {1, 10};

    // 2. Create input tensor from the observation vector
    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        memoryInfo, 
        const_cast<float*>(observation.data()), 
        observation.size(), 
        inputShape.data(), 
        inputShape.size()
    );

    // 3. Run Inference
    auto outputTensors = session.Run(
        Ort::RunOptions{nullptr}, 
        &inputName, &inputTensor, 1, 
        &outputName, 1
    );

    // 4. Extract results (Action vector: [F_norm, Tau_norm])
    float* outputArr = outputTensors.front().GetTensorMutableData<float>();
    return {outputArr[0], outputArr[1]};
}

std::pair<float, float> RLController::computeControl(const State& s, float dt) {
    if (waypoints.size() < 2) return {0, 0};

    Waypoint p0 = waypoints[currentTargetIdx - 1];
    Waypoint p1 = waypoints[currentTargetIdx];

// 1. Vector Math (Identical to Python's np.dot logic)
    float dx = p1.x - p0.x;
    float dy = p1.y - p0.y;
    float segment_len = sqrtf(dx*dx + dy*dy);
    
    // Normalized path direction (path_direction in Python)
    float ux = dx / segment_len;
    float uy = dy / segment_len;

    // Normal vector (path_normal in Python: [-dy, dx])
    float nx = -uy;
    float ny = ux;

    // Relative position from start of segment
    float rx = s.x - p0.x;
    float ry = s.y - p0.y;

    // Cross-track error (np.dot(rel_pos, path_normal))
    float e = (rx * nx) + (ry * ny); 

    // Along-track progress (np.dot(rel_pos, path_direction))
    float along_track = (rx * ux) + (ry * uy);

    // FIX: Swap uy and ux to match the atan2(dx, dy) North-facing coordinate system
    float desired_heading = atan2f(ux, uy); 
    float deltaTheta = desired_heading - s.theta;
    
    // Proper angle wrapping (Matches np.arctan2(sin, cos))
    deltaTheta = atan2f(sinf(deltaTheta), cosf(deltaTheta));

    // NEW: Project global velocities into path-relative velocities
    float v_along = (s.vx * ux) + (s.vy * uy);
    float v_cross = (s.vx * nx) + (s.vy * ny);

    // 3. Prepare Observation (Same 10-D order)
    std::vector<float> obs = {
        sinf(deltaTheta), cosf(deltaTheta),
        v_along, v_cross, s.omega,
        s.phi, s.dphi,
        e,
        along_track / segment_len,
        segment_len - along_track
    };

    // 3. Execute Model Inference
    std::vector<float> action = runInference(obs);

    // 4. Waypoint Switching Logic
    float distToP1 = sqrtf(powf(p1.x - s.x, 2) + powf(p1.y - s.y, 2));
    if (distToP1 < WP_RADIUS && currentTargetIdx < (int)waypoints.size() - 1) {
        currentTargetIdx++;
    }

    // 5. Scale normalized actions (-1 to 1) back to physical constants
    float finalF = std::clamp(action[0], -1.0f, 1.0f) * F_MAX;
    float finalTau = std::clamp(action[1], -1.0f, 1.0f) * T_MAX;

    return {finalF, finalTau};
}