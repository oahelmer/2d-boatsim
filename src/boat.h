#ifndef BOAT_H
#define BOAT_H

#include <vector>

struct State {
    float x, vx, y, vy, theta, omega, phi, dphi;
};

class Boat {
public:
    State state;
    Boat();
    void update(float dt, float F, float tau);
    void draw();
private:
    State f(State s, float F, float tau);
};

#endif