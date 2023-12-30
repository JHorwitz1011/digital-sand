#include "ball.hpp"
#include <stdio.h>

float Ball::max = 5;
float Ball::dampen = .0001;
Ball::Ball() {
    x = 0;
    y = 0;
    vx = 0;
    vy = 0;
    r = 0;
}

void Ball::update(float ax, float ay, float dt) {
    updateLinear(x, vx, ax, dt);
    updateLinear(y, vy, ay, dt);
}

void Ball::updateLinear(float &p, float &v, float a, float dt) {
    v += dt*a;

    //dampening coeff (ensure system stability)
    // if(v > 0) {
    //     v -= dampen*dt;
    // } 
    // else if(v < 0) {
    //     v += dampen*dt;
    // }

    p += dt*v;

    if(p > max) {
        //handle collision
        p = max; //1st, limit position
        v *= -1; //2nd, flip velocity
    }
    else if (p < 0) {
        p = 0;
        v *= -1;
    }
}   


void Ball::print() {
    printf("Ball\n - r=%f\n - vx=%f\n - vy=%f\n - px=%f\n - py=%f\n", r, vx, vy, x, y);
}