// #include "pico/stdlib.h"
#include <stdio.h>
#include <unistd.h>

#include "ball.hpp"

int main() {
    Ball b = Ball();
    while(1) {
        // printf("hello world!\n");
        b.update(.002, .002, 1);
        b.print();
        usleep(10000);
        printf("%f\n", Ball::max);
    }
}