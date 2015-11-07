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

#define RESTITUTION_COEFFCIENT 0.5

#include "PhysicsObject.h"

typedef struct DS {
	float deltaAngVel;
	Vector deltaVel;
	Point deltaPosition;
} State;

typedef struct CP{
	float depth;
	Vector normal;
	Point location;
}CollisionPair;


/*returns the collision parameters variation for object A
 in a collision of both objects A and B, this method 
 calculates both coarse and refined collision*/
State collideObjects(PhysicsObject* a, PhysicsObject* b);

void runPhysics(float timeStep);