#ifndef SOLVER_H
#define SOLVER_H

#define _BSD_SOURCE

#ifdef __APPLE__
    #include "TargetConditionals.h"
    #ifdef TARGET_OS_MAC
        #include <GLUT/glut.h>
        #include <OpenGL/OpenGL.h>
    #endif
#else
#include <GL/glut.h>
#endif 

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "bmp/bmp.h"

/* macros */

#define GET_R(v) (((v >> 16) & 0xFF)/255.0f)
#define GET_G(v) (((v >> 8) & 0xFF)/255.0f)
#define GET_B(v) ((v & 0xFF)/255.0f)

#define ADD_R(v,y) ((((((v>>16)&0xFF) + y) % 0xFF)) << 16)
#define ADD_G(v,y) ((((((v>>8)&0xFF) + y) % 0xFF)) << 8)
#define ADD_B(v,y) (((v + y) % 0xFF))

#define IX(i,j) ((i)+(solver->N+2)*(j))
#define SWAP(x0,x) {float * tmp=x0;x0=x;x=tmp;}
#define FOR_EACH_CELL for ( i=1 ; i<=solver->N ; i++ ) { for ( j=1 ; j<=solver->N ; j++ ) {
#define END_FOR }}

#define MAXBUF 65536

#define PI 3.14159265

/* Enums */

typedef enum{ APP_DEMO=0, APP_INTERACTIVE=1, APP_TEST=2, APP_CLIENT=3 } app_mode_e;
typedef enum{ false=0, true=1} bool;

/* Structs */
typedef struct fluid_solver_t {
	uint32_t N;					//matrix size
	float dt, diff, visc;		//phys. coefficients
	float * u, * v, * u_prev, * v_prev;	//velocity fields
	float * dens, * dens_prev;	//density fields
} __attribute__((__packed__)) fluid_solver;

typedef struct fluid_app_t {
	float force, source;		//interactive force, source ammount
	uint32_t win_id;
	uint32_t win_x, win_y;
	uint32_t mouse_down[3];
	int omx, omy, mx, my;
	uint32_t dvel;
	uint32_t mask_value;
	bool should_exit;
	app_mode_e app_mode;
	uint32_t client_ID;
	uint32_t client_PORT;
} __attribute__((__packed__)) fluid_app;

/* solver definitions (from solver.c) */

fluid_solver* solver_create (uint32_t N, float dt, float diff, float visc);
void solver_destroy ( fluid_solver* solver );

void solver_clear_data ( fluid_solver* solver );
void solver_set_initial_velocity ( fluid_solver* solver );
void solver_set_initial_density ( fluid_solver* solver );


void solver_dens_step ( fluid_solver* solver, float * x, float * x0);
void solver_vel_step ( fluid_solver* solver, float * u0, float * v0);

void solver_add_source ( fluid_solver* solver, float * x, float * s);
void solver_set_bnd ( fluid_solver* solver, uint32_t b, float * x );
void solver_lin_solve ( fluid_solver* solver, uint32_t b, float * x, float * x0, float a, float c );
void solver_diffuse ( fluid_solver* solver, uint32_t b, float * x, float * x0);
void solver_advect ( fluid_solver* solver, uint32_t b, float * d, float * d0, float * u, float * v);
void solver_project ( fluid_solver* solver, float * p, float * div );

#endif
