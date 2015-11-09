#include <stdio.h>
#include <stdlib.h>
#include <e-hal.h>
#include "PhysicsCalculator.h"
#include "messages.h"
#include <sys/time.h>

#define RAM_SIZE (0x8000)


typedef struct PEng{
    int count;
    PhysicsObject objects[CORES*MAX_ELEMENTS_PER_CORE];
} PhysicsEngine;



char addObject(PhysicsEngine* engine, PhysicsObject object){
    if(engine->count == MAX_ELEMENTS_PER_CORE*CORES){
        return 0;
    }
    if(!isObjectValid(&object)){
        return 0;
    }
    engine->objects[engine->count] = object;
    engine->count++;
    return 1;
}

char minimunCircles(PhysicsEngine* engine){
    int i;
    for(i=0; i<engine->count; i++){
        calculateMinimumCircle(&engine->objects[i]);
    }
}

void distributeObjectsToCores(PhysicsEngine* engine, e_epiphany_t* dev){
    int i, j, k;
    j=0;
    k=0;
    for(i=0; i<engine->count; i++){
        int address = COMMADDRESS_OBJECTS+(i/16)*sizeof(PhysicsObject);
        e_write(dev,k,j,address,&engine->objects[i],sizeof(PhysicsObject));
        j = j<3?j+1:0;
        k = j==3?(k<3?k+1:0):k;
    }

    int div = engine->count/16;
    int res = engine->count%16;
    for(j=0;j<4;j++){
        for(k=0;k<4;k++){
            char key = 4*j+k;
            char value = div + ((k+4*j)<res);
            printf("count(%d,%d)->%d %d\n",j,k,value, key);
            e_write(dev,j,k,COMMADDRESS_CORE_KEY, &key,sizeof(char));
            e_write(dev,j,k,COMMADDRESS_OBJECTS_COUNT, &value,sizeof(char));
        }
    }
    usleep(20000);

}


void clearMemory(){

	e_epiphany_t dev;
	char clear[0x6000] = {0};
	unsigned int i,j;

	for(i=0;i<4;i++){
		for(j=0;j<4; j++){
			e_open(&dev,i,j,1,1);
			e_reset_group(&dev);
			e_write(&dev,0,0,0x2000,clear,0x6000*sizeof(char));
			e_close(&dev);
		}
	}
	usleep(10000);

}


int main(){

    //EPIPHANY VARIABLES
	e_platform_t platform;
	e_epiphany_t dev;

	//DEBUG VARIABLES
	unsigned read_buffer[RAM_SIZE/4];
	unsigned read_data;
	unsigned addr;
	int i,j,k;

	char filename[9] = "logs.txt";
	FILE* file = fopen(filename,"w");

	//TIME VARIABLEs
	struct timeval initTime, endTime;
	long int timediff;

    //initialize the cores
	e_init(NULL);
	e_get_platform_info(&platform);

	clearMemory();

	e_open(&dev, 0,0,4,4);

	e_reset_group(&dev);
	//initialize physics objects and Physics Engine
	PhysicsEngine engine;
	engine.count = 0;

	PhysicsObject obj;
    obj.type = OBJECT_TYPE_POLYGON;
	addPointToObject(&obj,pointMake(0,0));
	addPointToObject(&obj,pointMake(2,-2));
	addPointToObject(&obj,pointMake(4,0));
	addPointToObject(&obj,pointMake(2,2));

	obj.rotationCenter = pointMake(2,0);
	obj.position = pointMake(0,0.99);
	obj.rotation = 0;
	obj.inverseInertia = 1;
	obj.inverseMass = 1;
	obj.linearVelocity = pointMake(0,-1);
	obj.angularVelocity = 0;

	addObject(&engine,obj);

	//-------
	PhysicsObject obj2;
    obj2.type = OBJECT_TYPE_POLYGON;
	addPointToObject(&obj2,pointMake(0,-1));
	addPointToObject(&obj2,pointMake(2,-4));
	addPointToObject(&obj2,pointMake(4,-1));

	obj2.rotationCenter = pointMake(2,-2);
	obj2.position = pointMake(0,0);
	obj2.rotation = 0;
	obj2.inverseInertia = 1;
	obj2.inverseMass = 1;
	obj2.linearVelocity = pointMake(0,0);
	obj2.angularVelocity = 0;

	addObject(&engine,obj2);

	//calculate minimun circles
	minimunCircles(&engine);

    printf("%x\n",sizeof(PhysicsObject));

    distributeObjectsToCores(&engine,&dev);

    printf("frame->%x\n",sizeof(Frame));
    char start = 1;
    for(i=0;i<4;i++){
        for(j=0;j<4;j++){
            e_load("epiphanyProgram.srec",&dev,i,j,E_TRUE);
            e_write(&dev,i,j,COMMADDRESS_READY, &start,sizeof(char));
            usleep(20000);
        }
    }

    usleep(3000000);

    Frame frm1, frm2;
    e_read(&dev,0,0,COMMADDRESS_FRAMES,&frm1,sizeof(Frame));
     e_read(&dev,0,1,COMMADDRESS_FRAMES,&frm2,sizeof(Frame));

    printf("Frame---\nvelocity(%g,%g)\nposition(%g,%g)\n",frm1.velocity.x,frm1.velocity.y,frm1.position.x,frm1.position.y);
    printf("Frame---\nvelocity(%g,%g)\nposition(%g,%g)\n",frm2.velocity.x,frm2.velocity.y,frm2.position.x,frm2.position.y);

    gettimeofday(&initTime,NULL);
    gettimeofday(&endTime, NULL);
    long long int TotalTime =endTime.tv_sec*1000+endTime.tv_usec/1000-initTime.tv_usec/1000-initTime.tv_sec*1000;
    printf("%lld\n", TotalTime);

    //-------------------------DUMP MEMORY -----------------------------
	//read all memory
	e_open(&dev, 0, 0, platform.rows, platform.cols);
	fprintf(file,"(ROW,COL)   ADDRESS   DATA\n");
	fprintf(file,"-----------------------------\n");
	for (i=0; i<(platform.rows); i++) {
		for (j=0; j<(platform.cols); j++) {
			for(k=0;k<RAM_SIZE/4;k++){
				addr=4*k;
				e_read(&dev, i, j, addr, &read_data, sizeof(int));
				read_buffer[k]=read_data;
			}
			for(k=0;k<RAM_SIZE/4;k++){
				fprintf(file,"(%2d,%2d)     0x%08x   0x%08x\n",i,j,k*4,read_buffer[k]);
			}
		}
	}

	fclose(file);
	e_close(&dev);
	e_finalize();

	return EXIT_SUCCESS;

}
