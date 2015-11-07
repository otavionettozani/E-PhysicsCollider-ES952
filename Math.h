//
//  Math.h
//  PhysicsCollision
//
//  Created by Otávio Netto Zani on 27/10/15.
//  Copyright © 2015 Otávio Netto Zani. All rights reserved.
//

#ifndef Math_h
#define Math_h

#include <stdio.h>

#endif /* Math_h */

#define PI 3.14159265359

//defines the format of a point
typedef struct PT{
	float x;
	float y;
}Point;

//only for a more readable code
typedef Point Vector;

//-----------------Basic Math functions--------------//

float Fabs(float num);

//binary search sqrt
float BS_sqrt(float value);

//taylor series sin
float TS9_sin(float angle);

//taylor series cos
float TS8_cos(float angle);

//constructor of point
Point pointMake(float x, float y);

//normalize a point
void normalizePoint(Point* p);

//distance between 2 points
float pointsDistance(Point a, Point b);

//rotate a vector
Vector rotateVector(Vector a, float angle);

