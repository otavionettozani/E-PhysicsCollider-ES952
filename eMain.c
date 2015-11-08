#include <stdio.h>
#include <stdlib.h>
#include "messages.h"
#include "PhysicsCalculator.h"
#include "e_lib.h"

int main(void){

    //cores memory banks
    int cores[16] = {CORE_0_0_MEM, CORE_0_1_MEM,CORE_0_2_MEM, CORE_0_3_MEM,
                    CORE_1_0_MEM, CORE_1_1_MEM,CORE_1_2_MEM, CORE_1_3_MEM,
                    CORE_2_0_MEM, CORE_2_1_MEM,CORE_2_2_MEM, CORE_2_3_MEM,
                    CORE_3_0_MEM, CORE_3_1_MEM,CORE_3_2_MEM, CORE_3_3_MEM};

    PhysicsObject* objects = (PhysicsObject*)COMMADDRESS_OBJECTS;
    Frame* frames = (Frame*)COMMADDRESS_FRAMES;
    char* count = (char*)COMMADDRESS_OBJECTS_COUNT;
    char* ready = (char*)COMMADDRESS_READY;
    char* halfReady = (char*)COMMADDRESS_HALF_READY;
    char* key = (char*)COMMADDRESS_CORE_KEY;

    int i, j, k;
    //State a;
    //collideObjects(&objects[0],&objects[1], &a);

    while(!*ready);

    for(i=0; i<*count; i++){ //foreach object inside this core
        //calculate a timestep
        frames[i].index = *key +16*i;
        frames[i].rotation = objects[i].rotation+objects[i].angularVelocity*TIMESTEP;
        frames[i].position.x = objects[i].position.x+objects[i].linearVelocity.x*TIMESTEP;
        frames[i].position.x = objects[i].position.y+objects[i].linearVelocity.y*TIMESTEP;

        for(j=0;j<16;j++){//foreach core
            //get the pointer to objects and count of that core
            char* coreCount = (char*)(cores[j] | COMMADDRESS_OBJECTS_COUNT);
            PhysicsObject* coreObjects =  (PhysicsObject*)(cores[j] | COMMADDRESS_OBJECTS);
            char* coreKey = (char*)(cores[j] | COMMADDRESS_CORE_KEY);
            for(k=0;k<*coreCount;k++){ //foreach object at target core
                if(*key+16*i == *coreKey+16*k){// if the object is itself
                    continue;
                }
                //calculate the collision
            }
        }
    }

    //frames[0].index = -1;
    //frames[0].rotation = -1;
    //frames[0].position.x = 2;
    //frames[0].position.y = 3;

    return EXIT_SUCCESS;

}
