#include <stdio.h>
#include <stdlib.h>
#include <e-hal.h>
#include "PhysicsCalculator.h"
#include "messages.h"
#include <sys/time.h>

#define RAM_SIZE (0x8000)

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
	//initialize physics objects
	PhysicsObject objs[3];
    objs[0].type = OBJECT_TYPE_POLYGON;
	addPointToObject(&objs[0],pointMake(0,0));
	addPointToObject(&objs[0],pointMake(2,-2));
	addPointToObject(&objs[0],pointMake(4,0));
	addPointToObject(&objs[0],pointMake(2,2));

	objs[0].rotationCenter = pointMake(2,0);
	objs[0].position = pointMake(0,0);
	objs[0].rotation = 0;
	objs[0].inverseInertia = 1;
	objs[0].inverseMass = 1;
	objs[0].linearVelocity = pointMake(0,-1);
	objs[0].angularVelocity = 0;
	calculateMinimumCircle(&objs[0]);
	//-------
    objs[1].type = OBJECT_TYPE_POLYGON;
	addPointToObject(&objs[1],pointMake(0,-1));
	addPointToObject(&objs[1],pointMake(2,-4));
	addPointToObject(&objs[1],pointMake(4,-1));

	objs[1].rotationCenter = pointMake(2,-2);
	objs[1].position = pointMake(0,0);
	objs[1].rotation = 0;
	objs[1].inverseInertia = 1;
	objs[1].inverseMass = 1;
	objs[1].linearVelocity = pointMake(0,0);
	objs[1].angularVelocity = 0;
	calculateMinimumCircle(&objs[1]);

    printf("%x\n",sizeof(PhysicsObject));
    for(i=0;i<4;i++){
        for(j=0;j<4;j++){
            e_write(&dev,i,j,COMMADDRESS_OBJECTS,&objs[0],2*sizeof(PhysicsObject));
            usleep(20000);
        }
    }


    for(i=0;i<4;i++){
        for(j=0;j<4;j++){
            e_load("epiphanyProgram.srec",&dev,i,j,E_TRUE);
            usleep(20000);
        }
    }


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
