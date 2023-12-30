#pragma once
#include <stdio.h>

class Ball {
    public:
        //size 
        float r;

        //pos
        float x; //0-5 led distances, roughly 4 inches
        float y; //0-5 led distances, roughly 4 inches
        static float max;
        static float dampen;

        //velocity
        float vx; //mm/s
        float vy; //mm/s

        /**
         * Updates velocity and position from acceleration
         * 
        */
        void update(float ax, float ay, float dt);
        
        /** 
         * prints current radius, x, y, vx, and vy.
        */
        void print();

        /**
         * default constructor
        */
        Ball();


    private:
        void updateLinear(float &p, float &v, float a, float dt);
};


