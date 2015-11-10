#include <stdio.h>
#include <stdlib.h>
#include "messages.h"
#include "PhysicsCalculator.h"
#include "e_lib.h"

#define OnVelocity (0)
#define EndVelocity (1)
#define OnCollision (2)
#define EndCollision (3)

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
    char nextCore = *key == 0x0f?0x00: *key+1;
    char* nextCoreReady = (char*)(cores[nextCore] | COMMADDRESS_HALF_READY);

    int i, j, k, frameCounter, lastFrame = 0, frameNumber =0;
    //State a;
    //collideObjects(&objects[0],&objects[1], &a);

    while(!*ready);

    while(1){
        *halfReady = OnVelocity;

        for(i=0; i<*count; i++){ //foreach object inside this core
            //calculate a timestep
            objects[i].rotation = objects[i].rotation+objects[i].angularVelocity*TIMESTEP;
            objects[i].position.x = objects[i].position.x+objects[i].linearVelocity.x*TIMESTEP;
            objects[i].position.y = objects[i].position.y+objects[i].linearVelocity.y*TIMESTEP;
        }
        //synch

        *halfReady = EndVelocity;
        char synch1 = 1;
        while(synch1){//wait for all cores to be at EndVelocity
            synch1 = 0;
            for(i=0;i<CORES;i++){
                char* coreReady = (char*)(cores[i] | COMMADDRESS_HALF_READY);
                char* coreCount = (char*)(cores[i] | COMMADDRESS_OBJECTS_COUNT);
                if(*coreCount ==0){//if core does not have a object, ignore it
                    continue;
                }
                if(*coreReady != EndVelocity){
                    synch1 = 1;
                    break;
                }
            }
            if(*halfReady == OnCollision){
                break;
            }
        }
        *halfReady = OnCollision;
        if(*nextCoreReady!=OnCollision && *nextCoreReady!=EndCollision ){
            *nextCoreReady = OnCollision;
        }


        //calculating the collisions
        State state;
        for(i=0; i<*count; i++){ //foreach object inside this core
            state.deltaAngVel = 0;
            state.deltaPosition.x = 0;
            state.deltaPosition.y = 0;
            state.deltaVel.x = 0;
            state.deltaVel.y = 0;
            int currentFrame = i;//frameCounter*(*count)+i;
            frames[currentFrame].index = *key +16*i;
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
                    collideObjects(&objects[i],&coreObjects[k],&state);
                }
            }
            frames[currentFrame].rotation = objects[i].rotation;
            frames[currentFrame].angVelocity = objects[i].angularVelocity + state.deltaAngVel;
            frames[currentFrame].position.x = objects[i].position.x + state.deltaPosition.x;
            frames[currentFrame].position.y = objects[i].position.y + state.deltaPosition.y;
            frames[currentFrame].velocity.x = objects[i].linearVelocity.x + state.deltaVel.x;
            frames[currentFrame].velocity.y = objects[i].linearVelocity.y + state.deltaVel.y;
            frames[currentFrame].frameNumber = frameNumber;
        }

        //synch
        *halfReady = EndCollision;
        char synch2 = 1;
        while(synch2){//wait for all cores to be at EndVelocity
            synch2 = 0;
            for(i=0;i<CORES;i++){
                char* coreReady = (char*)(cores[i] | COMMADDRESS_HALF_READY);
                char* coreCount = (char*)(cores[i] | COMMADDRESS_OBJECTS_COUNT);
                if(*coreCount ==0){//if core does not have a object, ignore it
                    continue;
                }
                if(*coreReady != EndCollision){
                    synch2 = 1;
                    break;
                }
            }
            if(*halfReady == OnVelocity){
                break;
            }
        }
        *halfReady = OnVelocity;
        if(*nextCoreReady!=OnVelocity && *nextCoreReady!=EndVelocity ){
            *nextCoreReady = OnVelocity;
        }

        //update objects
        for(i=0; i<*count; i++){ //foreach object inside this core
            int currentFrame = i;//frameCounter*(*count)+i;
            objects[i].angularVelocity = frames[currentFrame].angVelocity;
            objects[i].position.x = frames[currentFrame].position.x;
            objects[i].position.y = frames[currentFrame].position.y;
            objects[i].linearVelocity.x = frames[currentFrame].velocity.x;
            objects[i].linearVelocity.y = frames[currentFrame].velocity.y;
        }
        frameNumber++;
    }

    return EXIT_SUCCESS;

}
