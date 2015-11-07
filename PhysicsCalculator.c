//
//  PhysicsCalculator.c
//  PhysicsCollision
//
//  Created by Otávio Netto Zani on 29/10/15.
//  Copyright © 2015 Otávio Netto Zani. All rights reserved.
//

#include "PhysicsCalculator.h"



State physicsResolution(PhysicsObject*a, PhysicsObject*b, CollisionPair pair){
	//first we find the impulse in our collision
	float impulse;
	
	Point centerA = worldPosition(a->rotationCenter, *a);
	Point centerB = worldPosition(b->rotationCenter, *b);
	Point RA = pointMake(pair.location.x-centerA.x, pair.location.y-centerA.y);
	Point RB = pointMake(pair.location.x-centerB.x, pair.location.y-centerB.y);
	
	Vector VAP = pointMake(a->linearVelocity.x-a->angularVelocity*RA.y, a->linearVelocity.y+a->angularVelocity*RA.x);
	Vector VBP = pointMake(b->linearVelocity.x-b->angularVelocity*RB.y, b->linearVelocity.y+b->angularVelocity*RB.x);
	Vector VAB = pointMake(VAP.x - VBP.x, VAP.y - VBP.y);
	
	float crossRANsquare = RA.x*pair.normal.y-pair.normal.x*RA.y;
	crossRANsquare = crossRANsquare*crossRANsquare;
	float crossRBNsquare = RB.x*pair.normal.y-pair.normal.x*RB.y;
	crossRBNsquare = crossRBNsquare*crossRBNsquare;
	
	impulse = a->inverseMass + b->inverseMass + a->inverseInertia*crossRANsquare + b->inverseInertia*crossRBNsquare;
	impulse = -(1+RESTITUTION_COEFFCIENT)*(VAB.x*pair.normal.x+VAB.y*pair.normal.y)/impulse;
	
	
	State DeltaState;
	DeltaState.deltaVel = pointMake(impulse*a->inverseMass*pair.normal.x,impulse*a->inverseMass*pair.normal.y);
	DeltaState.deltaAngVel = (RA.x*pair.normal.y-RA.y*pair.normal.x)*a->inverseInertia*impulse;
	
	float deltaPosModule = a->inverseMass*pair.depth/(a->inverseMass+b->inverseMass);
	DeltaState.deltaPosition = pointMake(deltaPosModule*pair.normal.x, deltaPosModule*pair.normal.y);
	
	return DeltaState;
}

char coarseCollision(PhysicsObject*a, PhysicsObject*b){
	Point aCenter = worldPosition(a->minimumCirclePosition, *a);
	Point bCenter = worldPosition(b->minimumCirclePosition, *b);
	float centerDistance = pointsDistance(aCenter, bCenter);
	float radiusSum = a->minimumCircleRadius + b->minimumCircleRadius;
	return centerDistance<radiusSum;
}

State collideObjects(PhysicsObject* a, PhysicsObject* b){
	State state;
	state.deltaAngVel = 0;
	state.deltaPosition = pointMake(0, 0);
	state.deltaVel = pointMake(0, 0);
	
	CollisionPair pairs[2*PHYSICS_OBJECT_MAXIMUM_POINTS];
	if(!coarseCollision(a, b)){
		return state;
	}
	
	//if a coarse collision happened, we will search for a refined one in each of A's edges.
	//in this step we try to find every point where B has hitted A and A has hitted B
	int Acurrent, Anext, Abefore, Bcurrent, Bnext, pairCount=0;
	
	//first case---------------------------------------------------------------------------
	/* 
	 PA1     PA3
	 \      /
	  \    /
  PB2---------PB1
	    \/
		PA2
	 
	V=A --=B
	*/
	for (Acurrent=0; Acurrent<a->format.polygonInfo.count; Acurrent++) {
		/*Given 3 sequenced vertex, we create two vectors, one from the
		 first to the second vertex and other from the second to the
		 third vertex.*/
		Anext = Acurrent==a->format.polygonInfo.count-1?0:Acurrent+1;
		Abefore = Acurrent==0?a->format.polygonInfo.count-1:Acurrent-1;
		Point PA1 = worldPosition(a->format.polygonInfo.points[Abefore],*a);
		Point PA2 = worldPosition(a->format.polygonInfo.points[Acurrent],*a);
		Point PA3 = worldPosition(a->format.polygonInfo.points[Anext],*a);
		Vector A1 = pointMake(PA2.x-PA1.x, PA2.y-PA1.y);
		Vector A2 = pointMake(PA3.x-PA2.x, PA3.y-PA2.y);

		pairs[pairCount].depth = 0;
		pairs[pairCount].normal = pointMake(0, 0);
		for (Bcurrent = 0; Bcurrent<b->format.polygonInfo.count; Bcurrent++) {
			/*for each edge of B, we will find if any of the edges are hitted by both
			 edges defined before, if thats the case, then A hitted B on that point
			 If both edges of A hitted more than one edge of B, we will keep the highest
			 depth*/
			Bnext = Bcurrent==b->format.polygonInfo.count-1?0:Bcurrent+1;
			Point PB1 = worldPosition(b->format.polygonInfo.points[Bcurrent], *b);
			Point PB2 = worldPosition(b->format.polygonInfo.points[Bnext], *b);
			Vector B = pointMake(PB2.x-PB1.x, PB2.y-PB1.y);
			Vector I1 = pointMake(PB1.x-PA1.x, PB1.y-PA1.y);
			Vector I2 = pointMake(PB1.x-PA2.x, PB1.y-PA2.y);
			
			float crossBI1 = B.x*I1.y-B.y*I1.x;
			float crossBI2 = B.x*I2.y-B.y*I2.x;
			float crossAI1 = A1.x*I1.y-A1.y*I1.x;
			float crossAI2 = A2.x*I2.y-A2.y*I2.x;
			float crossBA1 = B.x*A1.y-B.y*A1.x;
			float crossBA2 = B.x*A2.y-B.y*A2.x;
			
			crossBA1=crossBA1==0?0.0001:crossBA1;
			crossBA2=crossBA2==0?0.0001:crossBA2;
			
			float t1 = crossAI1/crossBA1;
			float w1 = crossBI1/crossBA1;
			float t2 = crossAI2/crossBA2;
			float w2 = crossBI2/crossBA2;
			
			if(t1>0 && t1<1 && w1>0 && w1<1 && t2>0 && t2<1 && w2>0 && w2<1){
				//we do have a collision
				
				Vector normal = rotateVector(B, -PI*0.5); //rotate the edge by -90 degrees, so that it points outside
				normalizePoint(&normal);
				Point collisionPoint=pointMake(((PA1.x+w1*A1.x)+(PA2.x+w2*A2.x))*0.5, ((PA1.y+w1*A1.y)+(PA2.y+w2*A2.y))*0.5);
				float depth = (PA2.x-collisionPoint.x)*normal.x+(PA2.y-collisionPoint.y)*normal.y;
				depth = -depth;
				if(Fabs(depth)>Fabs(pairs[pairCount].depth)){
					pairs[pairCount].depth = depth;
					pairs[pairCount].normal = normal;
					pairs[pairCount].location=collisionPoint;
				}
			}
		}
		if(Fabs(pairs[pairCount].depth)>0){
			pairCount++;
		}
	}
	
	//second case---------------------------------------------------------------------------
	/*
	 PA1     PA3
	 \      /
	  \    /
  PB2---------PB1
	    \/
		PA2
	 
	 V=B --=A
	 */
	//Although we did not change the names, A variables now refers to B while B variables refer to A
	for (Acurrent=0; Acurrent<b->format.polygonInfo.count; Acurrent++) {
		/*Given 3 sequenced vertex, we create two vectors, one from the
		 first to the second vertex and other from the second to the
		 third vertex.*/
		Anext = Acurrent==b->format.polygonInfo.count-1?0:Acurrent+1;
		Abefore = Acurrent==0?b->format.polygonInfo.count-1:Acurrent-1;
		Point PA1 = worldPosition(b->format.polygonInfo.points[Abefore],*b);
		Point PA2 = worldPosition(b->format.polygonInfo.points[Acurrent],*b);
		Point PA3 = worldPosition(b->format.polygonInfo.points[Anext],*b);
		Vector A1 = pointMake(PA2.x-PA1.x, PA2.y-PA1.y);
		Vector A2 = pointMake(PA3.x-PA2.x, PA3.y-PA2.y);
		
		pairs[pairCount].depth = 0;
		pairs[pairCount].normal = pointMake(0, 0);
		for (Bcurrent = 0; Bcurrent<a->format.polygonInfo.count; Bcurrent++) {
			/*for each edge of A, we will find if any of the edges are hitted by both
			 edges defined before, if thats the case, then A hitted B on that point
			 If both edges of B hitted more than one edge of A, we will keep the highest
			 depth*/
			Bnext = Bcurrent==a->format.polygonInfo.count-1?0:Bcurrent+1;
			Point PB1 = worldPosition(a->format.polygonInfo.points[Bcurrent], *a);
			Point PB2 = worldPosition(a->format.polygonInfo.points[Bnext], *a);
			Vector B = pointMake(PB2.x-PB1.x, PB2.y-PB1.y);
			Vector I1 = pointMake(PB1.x-PA1.x, PB1.y-PA1.y);
			Vector I2 = pointMake(PB1.x-PA2.x, PB1.y-PA2.y);
			
			float crossBI1 = B.x*I1.y-B.y*I1.x;
			float crossBI2 = B.x*I2.y-B.y*I2.x;
			float crossAI1 = A1.x*I1.y-A1.y*I1.x;
			float crossAI2 = A2.x*I2.y-A2.y*I2.x;
			float crossBA1 = B.x*A1.y-B.y*A1.x;
			float crossBA2 = B.x*A2.y-B.y*A2.x;
			
			crossBA1=crossBA1==0?0.0001:crossBA1;
			crossBA2=crossBA2==0?0.0001:crossBA2;
			
			float t1 = crossAI1/crossBA1;
			float w1 = crossBI1/crossBA1;
			float t2 = crossAI2/crossBA2;
			float w2 = crossBI2/crossBA2;
			
			if(t1>0 && t1<1 && w1>0 && w1<1 && t2>0 && t2<1 && w2>0 && w2<1){
				//we do have a collision
				
				Vector normal = rotateVector(B, -PI*0.5); //rotate the edge by -90 degrees, so that it points outside
				normalizePoint(&normal);
				Point collisionPoint=pointMake(((PA1.x+w1*A1.x)+(PA2.x+w2*A2.x))*0.5, ((PA1.y+w1*A1.y)+(PA2.y+w2*A2.y))*0.5);
				float depth = (PA2.x-collisionPoint.x)*normal.x+(PA2.y-collisionPoint.y)*normal.y;
				if(Fabs(depth)>Fabs(pairs[pairCount].depth)){
					pairs[pairCount].depth = depth;
					pairs[pairCount].normal = normal;
					pairs[pairCount].location=collisionPoint;
				}
			}
		}
		if(Fabs(pairs[pairCount].depth)>0){
			pairCount++;
		}
	}
	
	//third case---------------------------------------------------------------------------
	/*
	PA1     PA3
	 \  PB2 /
	  \ /\ /
       X  X
	  / \/ \
	 /  PA2 \
   PB3      PB1
	 
	 V=A /\=B
	 */
	int Bbefore;
	for (Acurrent=0; Acurrent<a->format.polygonInfo.count; Acurrent++) {
		/*Given 3 sequenced vertex, we create two vectors, one from the
		 first to the second vertex and other from the second to the
		 third vertex.*/
		Anext = Acurrent==a->format.polygonInfo.count-1?0:Acurrent+1;
		Abefore = Acurrent==0?a->format.polygonInfo.count-1:Acurrent-1;
		Point PA1 = worldPosition(a->format.polygonInfo.points[Abefore],*a);
		Point PA2 = worldPosition(a->format.polygonInfo.points[Acurrent],*a);
		Point PA3 = worldPosition(a->format.polygonInfo.points[Anext],*a);
		Vector A12 = pointMake(PA2.x-PA1.x, PA2.y-PA1.y);
		Vector A23 = pointMake(PA3.x-PA2.x, PA3.y-PA2.y);
		
		pairs[pairCount].depth = 0;
		pairs[pairCount].normal = pointMake(0, 0);
		for (Bcurrent = 0; Bcurrent<b->format.polygonInfo.count; Bcurrent++) {
			/*for each pair of edge of B, we will find if any of the edges are hitted as shown above*/
			Bnext = Bcurrent==b->format.polygonInfo.count-1?0:Bcurrent+1;
			Bbefore = Bcurrent==0?b->format.polygonInfo.count-1:Bcurrent-1;
			Point PB1 = worldPosition(b->format.polygonInfo.points[Bbefore], *b);
			Point PB2 = worldPosition(b->format.polygonInfo.points[Bcurrent], *b);
			Point PB3 = worldPosition(b->format.polygonInfo.points[Bnext], *b);
			Vector B12 = pointMake(PB2.x-PB1.x, PB2.y-PB1.y);
			Vector B23 = pointMake(PB3.x-PB2.x, PB3.y-PB2.y);
			Vector IA12B23 = pointMake(PB2.x-PA1.x, PB2.y-PA1.y);
			Vector IA23B12 = pointMake(PB1.x-PA2.x, PB1.y-PA2.y);
			
			float crossB12I = B12.x*IA23B12.y-B12.y*IA23B12.x;
			float crossB23I = B23.x*IA12B23.y-B23.y*IA12B23.x;
			float crossA12I = A12.x*IA12B23.y-A12.y*IA12B23.x;
			float crossA23I = A23.x*IA23B12.y-A23.y*IA23B12.x;
			float crossB12A23 = B12.x*A23.y-B12.y*A23.x;
			float crossB23A12 = B23.x*A12.y-B23.y*A12.x;
			
			crossB23A12=crossB23A12==0?0.0001:crossB23A12;
			crossB12A23=crossB12A23==0?0.0001:crossB12A23;
			
			float t1 = crossA12I/crossB23A12;
			float w1 = crossB23I/crossB23A12;
			float t2 = crossA23I/crossB12A23;
			float w2 = crossB12I/crossB12A23;
			
			if(t1>0 && t1<1 && w1>0 && w1<1 && t2>0 && t2<1 && w2>0 && w2<1){
				//we do have a collision
				Point collision1 = pointMake((PA1.x+w1*A12.x), (PA1.y+w1*A12.y));
				Point collision2 = pointMake((PA2.x+w2*A23.x), (PA2.y+w2*A23.y));
				Vector c12 = pointMake(collision2.x-collision1.x, collision2.y-collision1.y);
				Vector normal = rotateVector(c12, -PI*0.5); //rotate the edge by -90 degrees, so that it points outside
				normalizePoint(&normal);
				Point collisionPoint=pointMake((collision1.x+collision2.x)*0.5, (collision1.y+collision2.y)*0.5);
				float depth = (PA2.x-PB2.x)*normal.x+(PA2.y-PB2.y)*normal.y;
				depth = -depth;
				pairs[pairCount].depth = depth;
				pairs[pairCount].normal = normal;
				pairs[pairCount].location=collisionPoint;
				pairCount++;
			}
		}
	}
	
	int currentCollision;
	//solves the physics part ot the collision for each collision that has happened

	for (currentCollision =0; currentCollision<pairCount; currentCollision++) {
		State st = physicsResolution(a,b,pairs[currentCollision]);
		state.deltaAngVel += st.deltaAngVel;
		state.deltaPosition=pointMake(st.deltaPosition.x+state.deltaPosition.x, st.deltaPosition.y+state.deltaPosition.y);
		state.deltaVel=pointMake(st.deltaVel.x+state.deltaVel.x, st.deltaVel.y+state.deltaVel.y);
	}
	state.deltaAngVel += a->angularVelocity;
	state.deltaVel = pointMake(a->linearVelocity.x+state.deltaVel.x, a->linearVelocity.y+state.deltaVel.y);
	state.deltaPosition = pointMake(a->position.x+state.deltaPosition.x, a->position.y+state.deltaPosition.y);
	
	return state;
}