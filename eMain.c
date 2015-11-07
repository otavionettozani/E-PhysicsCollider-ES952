#include <stdio.h>
#include <stdlib.h>
#include "messages.h"
#include "PhysicsCalculator.h"
#include "e_lib.h"

int main(void){


    PhysicsObject* objects = (PhysicsObject*)COMMADDRESS_OBJECTS;
    float* end = (float*)0x6400;

    Point aCenter = worldPosition(&objects[0].minimumCirclePosition, &objects[0]);

    end[0] = -1;
    end[1] = aCenter.x;
    end[2] = aCenter.y;
    end[3] = -1;

    return EXIT_SUCCESS;

}
