//
//  PhysicsObject.c
//  PhysicsCollision
//
//  Created by Otávio Netto Zani on 26/10/15.
//  Copyright © 2015 Otávio Netto Zani. All rights reserved.
//

#include "PhysicsObject.h"
#include <stdlib.h>


//--------------------------Building Objects--------------------------//

char addPointToObject(PhysicsObject* object, Point point){
	
	if(object->type == OBJECT_TYPE_CIRCLE){
		printf("Given Object is a Circle\n");
		return 0;
	}
	if(object->format.polygonInfo.count >= PHYSICS_OBJECT_MAXIMUM_POINTS){
		printf("Given Object already reached limit of points\n");
		return 0;
	}
	
	object->format.polygonInfo.points[object->format.polygonInfo.count] = point;
	object->format.polygonInfo.count++;
	
	return 1;
}

//-----------------Geometry Validation and Generics--------------------//

char isObjectValid(PhysicsObject* object){
	if(object->type == OBJECT_TYPE_CIRCLE && object->format.radius > 0){
		return 1;
	}else if(object->type == OBJECT_TYPE_POLYGON && object->format.polygonInfo.count > 2){
		return 1;
	}
	return 0;
}

Point worldPosition(Point point, PhysicsObject object){
	
	float c1 = TS8_cos(object.rotation);
	float s1 = TS9_sin(object.rotation);
	float offsetx = object.rotationCenter.x*(1-c1)+object.rotationCenter.y*s1+object.position.x;
	float offsety = object.rotationCenter.y*(1-c1)-object.rotationCenter.x*s1+object.position.y;
	
	return pointMake(c1*point.x-s1*point.y+offsetx, s1*point.x+c1*point.y+offsety);
	
}

//--------------------------Pre-Physics Geometry Calculations--------------------------//

//copies the points array as an aux function to calculate minimum circle
Point* copyPoints(int size, Point* points){
	Point* retPoints = (Point*)malloc(sizeof(Point)*PHYSICS_OBJECT_MAXIMUM_POINTS);
	int i=0;
	for(i=0; i<size ; i++){
		retPoints[i] = points[i];
	}
	
	return retPoints;
}

//verifies if a point is inside a circle
char isPointInsideCircle(Point point, float radius, Point position){
	if(pointsDistance(point, position)<=radius){
		return 1;
	}
	return 0;
}


//Welzl final recursion - calculates the actual circle
void minimumDisk(Point* P, Point* R, int sizeP, int sizeR, float* radius, Point* position){
	
	if(sizeP==0 || sizeR==3){
		if(sizeR==1){
			*radius = 0;
			*position = R[0];
			return;
		}
		if(sizeR==2){
			*position = pointMake((R[0].x+R[1].x)/2., (R[0].y+R[1].y)/2.);
			*radius = pointsDistance(R[0], R[1])/2.;
			return;
		}
		if(sizeR==3){
			
			Point P1 = pointMake((R[0].x+R[1].x)/2., (R[0].y+R[1].y)/2.);
			Point AB = pointMake((R[1].x-R[0].x)/2., (R[1].y-R[0].y)/2.);
			Point BC = pointMake((R[2].x-R[1].x)/2., (R[2].y-R[1].y)/2.);
			Point AC = pointMake((R[2].x-R[0].x)/2., (R[2].y-R[0].y)/2.);
			Point PR = pointMake(-AB.y,AB.x);
			
			float w = -1./2.*(BC.x*AC.x+BC.y*AC.y)/(BC.x*AB.y-BC.y*AB.x);
			
			*position = pointMake(P1.x+w*PR.x, P1.y+w*PR.y);
			
			*radius = pointsDistance(*position, R[0]);
			
			return;
		}
	}
	
	Point* P2 = copyPoints(sizeP, P);
	Point* R2 = copyPoints(sizeR, R);
	
	
	sizeP--;
	Point point = P[sizeP];

	minimumDisk(P2, R2, sizeP, sizeR, radius, position);
	
	if(isPointInsideCircle(point, *radius, *position)){
		free(R2);
		free(P2);
		return;
	}
	
	
	R2[sizeR] = point;
	
	sizeR++;
	
	minimumDisk(P2, R2, sizeP, sizeR, radius, position);
	
	free(R2);
	free(P2);
	return;
	
	
}

//Welzl algorithm
void minimumCircle(float* radius, Point* position, Point* points,int size){

	if(size == 0){
		*radius = 0;
		*position = pointMake(0, 0);
		return;
	}
	
	size--;
	Point* evaluationPoint = &points[size];
	
	minimumCircle(radius, position, points, size);
	
	if(isPointInsideCircle(*evaluationPoint, *radius, *position)){
		return;
	}
	
	Point* P = copyPoints(size, points);
	Point* R = copyPoints(1, evaluationPoint);

	minimumDisk(P, R, size, 1, radius, position);
	
	free(P);
	free(R);
	
	return;
	
}

void calculateMinimumCircle(PhysicsObject* object){
	
	float radius;
	Point position;
	Point* copy = copyPoints(object->format.polygonInfo.count, object->format.polygonInfo.points);
	minimumCircle(&radius,&position,copy,object->format.polygonInfo.count);
	free(copy);
	object->minimumCirclePosition = position;
	object->minimumCircleRadius = radius;
	return;
}

//auxiliar function to isPointInsideTriangle
float signInPoints(Point p1, Point p2, Point p3){
	return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}

//returns 1 if P is inside triangle defined by T1 T2 and T3, 0 otherwise
char isPointInsideTriangle(Point t1, Point t2, Point t3, Point p){
	char b1, b2, b3;
	
	b1 = signInPoints(p, t1, t2) < 0.0f;
	b2 = signInPoints(p, t2, t3)  < 0.0f;
	b3 = signInPoints(p, t3, t1)  < 0.0f;
	
	return ((b1 == b2) && (b2 == b3));
}

//returns how many triangles were inserted in the last recursion step
int trueSeparateTriangles(Point* points, PhysicsObject* object, int currentTriangle, int size){
	float* signals = (float*)malloc(size*sizeof(float));
	int i=0;
	//verifies if any of the points is a concave point. we assumed that the points are given at counter-clockwise order
	for (i=0; i<size; i++) {
		int j = i-1<0?size-1:i-1;
		int k = i+1>=size?0:i+1;
		
		Point p1 = points[j];
		Point p2 = points[i];
		Point p3 = points[k];
		
		signals[i] =(p2.x-p1.x)*(p3.y-p2.y)-(p3.x-p2.x)*(p2.y-p1.y);
	}
	
	char isConvex = 1;
	int removable = 0;
	for(i=0;i<size;i++){
		if(signals[i]<0){//is a concave point, so the polygon is concave
			isConvex = 0;
			int j= i-1<0?size-1:i-1;
			if(signals[j]>0){//the point just before the concave one is an option for a triangle
				int k = j==0?size-1:j-1;
				char hasPointInside = 0;
				int l=0;
				for(l=0;l<size;l++){//check if the triangle has a point inside it
					if(l == i || l == j || l==k){
						continue;
					}
					if(isPointInsideTriangle(points[i], points[j], points[k], points[l])){
						hasPointInside = 1;
						break;
					}
				}
				if(hasPointInside){
					continue;
				}
				//if everything is ok, add the triangle to the structure
				object->format.polygonInfo.triangles[currentTriangle][0] = points[i];
				object->format.polygonInfo.triangles[currentTriangle][1] = points[j];
				object->format.polygonInfo.triangles[currentTriangle][2] = points[k];
				currentTriangle++;
				removable = j;
				break;
			}
			j = i+1>=size?0:i+1;
			if(signals[j]>0){//the point just after the concave one is an option for a triangle
				int k = j==size-1?0:j+1;
				char hasPointInside = 0;
				int l=0;
				for(l=0;l<size;l++){//check if the triangle has a point inside it
					if(l == i || l == j || l==k){
						continue;
					}
					if(isPointInsideTriangle(points[i], points[j], points[k], points[l])){
						hasPointInside = 1;
						break;
					}
				}
				if(hasPointInside){
					continue;
				}
				//if everything is ok, add the triangle to the structure
				object->format.polygonInfo.triangles[currentTriangle][0] = points[i];
				object->format.polygonInfo.triangles[currentTriangle][1] = points[j];
				object->format.polygonInfo.triangles[currentTriangle][2] = points[k];
				currentTriangle++;
				removable = j;
				break;
			}
		}
	}
	
	free(signals);
	
	//in the case object is convex, just fix a point and trace a line to all other points.
	if(isConvex){
		int maxTriangles = size -2;
		for(i=0;i<maxTriangles; i++){
			object->format.polygonInfo.triangles[currentTriangle][0] = points[0];
			object->format.polygonInfo.triangles[currentTriangle][1] = points[i+1];
			object->format.polygonInfo.triangles[currentTriangle][2] = points[i+2];
			currentTriangle++;
		}
		
		return size-2;
	}
	//if is a concave structure
	
	Point* points2 = copyPoints(size, points);
	for(i=removable;i<size-1;i++){
		points2[i] = points2[i+1];
	}
	
	trueSeparateTriangles(points2, object, currentTriangle, size-1);
	
	free(points2);
	
	return 1;
}

void separateTriangles(PhysicsObject* object){
	int size = object->format.polygonInfo.count;
	Point* points = copyPoints(size, object->format.polygonInfo.points);
	trueSeparateTriangles(points,object,0, size);
	free(points);
	return;
}
