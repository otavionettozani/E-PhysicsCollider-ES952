//
//  PhysicsCalculator.h
//  PhysicsCollision
//
//  Created by Otávio Netto Zani on 29/10/15.
//  Copyright © 2015 Otávio Netto Zani. All rights reserved.
//

#ifndef PhysicsCalculator_h
#define PhysicsCalculator_h

#include <stdio.h>

#endif /* PhysicsCalculator_h */

#define RESTITUTION_COEFFCIENT (0.5)

#define TIMESTEP (0.04166666)

#include "PhysicsObject.h"

#define MAX_ELEMENTS_PER_CORE (4)
#define CORES (16)

typedef struct DS {
	float deltaAngVel;
	Vector deltaVel;
	Point deltaPosition;
} State;

typedef struct CP{
	float depth;
	Vector normal;
	Point location;
} CollisionPair;

//one frame of the current object status
#define FRAMES_PER_STEP (10)
typedef struct FR{
    char index;
    int frameNumber;
    Point position;
    Vector velocity;
    float rotation;
    float angVelocity;
}__attribute__((aligned(8))) Frame;

/*returns the collision parameters variation for object A
 in a collision of both objects A and B, this method
 calculates both coarse and refined collision*/
void collideObjects(PhysicsObject* a, PhysicsObject* b, State* state);

void runPhysics(float timeStep);
