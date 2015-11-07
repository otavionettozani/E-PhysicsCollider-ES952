#include <stdio.h>
#include <stdlib.h>
#include "messages.h"
#include "PhysicsCalculator.h"
#include "e_lib.h"

int main(void){


    PhysicsObject* objects = (PhysicsObject*)COMMADDRESS_OBJECTS;
    float* end = (float*)0x6400;

    char a = coarseCollision(objects[0],objects[1]);
    end[0] = -1;
    end[1] = a;
    end[2] = 0;
    end[3] = -1;

    return EXIT_SUCCESS;

}
