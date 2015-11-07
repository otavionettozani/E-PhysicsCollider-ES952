//
//  Math.c
//  PhysicsCollision
//
//  Created by Otávio Netto Zani on 27/10/15.
//  Copyright © 2015 Otávio Netto Zani. All rights reserved.
//

#include "Math.h"
#define THRESHOLD 0.00001

float BS_sqrt(float value){
	float base =value<1?value:0, top=value<1?1:value;
	float answer = value/2;
	
	while (answer*answer-value<-THRESHOLD || answer*answer-value>THRESHOLD) {
		if(answer*answer<value){
			base = answer;
		}else{
			top	= answer;
		}
		answer = (base+top)/2.;
	}
	
	return answer;
}

float Fabs(float num){
	return num<0?-num:num;
}

float TS9_sin(float angle){
	while(angle<-PI){
		angle+=2*PI;
	}
	while(angle>PI){
		angle-=2*PI;
	}
	
	float angle2 = angle*angle;
	float angle3 = angle2*angle;
	float angle5 = angle2*angle3;
	float angle7 = angle2*angle5;
	float angle9 = angle2*angle7;
	
	return angle - 0.166666*angle3 + 0.008333*angle5 - 0.000198*angle7 + 0.000002*angle9;
}

float TS8_cos(float angle){
	while(angle<-PI){
		angle+=2*PI;
	}
	while(angle>PI){
		angle-=2*PI;
	}
	
	float angle2 = angle*angle;
	float angle4 = angle2*angle2;
	float angle6 = angle4*angle2;
	float angle8 = angle4*angle4;
	
	return 1 - 0.5*angle2 + 0.041666*angle4 - 0.001388*angle6 + 0.000024*angle8;
}

Point pointMake(float x, float y){
	Point a;
	a.x = x;
	a.y = y;
	return a;
}

void normalizePoint(Point* p){
	float length = BS_sqrt(p->x*p->x+p->y*p->y);
	p->x /= length;
	p->y /= length;
	return;
}

float pointsDistance(Point a, Point b){
	float deltax = a.x - b.x;
	float deltay = a.y - b.y;
	return BS_sqrt(deltax*deltax + deltay*deltay);
}

Vector rotateVector(Vector a, float angle){
	Vector result;
	float sin = TS9_sin(angle), cos = TS8_cos(angle);
	result.x = cos*a.x - sin*a.y;
	result.y = sin*a.x + cos*a.y;
	return result;
}