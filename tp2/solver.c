#include "solver.h"

fluid_solver* solver_create (uint32_t N, float dt, float diff, float visc){
	fluid_solver* solver = (fluid_solver*)malloc(sizeof(fluid_solver));
	solver->N			= N;
	solver->dt			= dt;
	solver->diff		= diff;
	solver->visc		= visc;
	uint32_t size = (N+2)*(N+2);
	solver->u			= (float *) malloc ( size*sizeof(float) );
	solver->v			= (float *) malloc ( size*sizeof(float) );
	solver->u_prev		= (float *) malloc ( size*sizeof(float) );
	solver->v_prev		= (float *) malloc ( size*sizeof(float) );
	solver->dens		= (float *) malloc ( size*sizeof(float) );	
	solver->dens_prev	= (float *) malloc ( size*sizeof(float) );
	if ( !solver->u || !solver->v || !solver->u_prev || !solver->v_prev || !solver->dens || !solver->dens_prev ) {
		fprintf ( stderr, "cannot allocate data\n" );
		return NULL;
	}
	solver_clear_data(solver);
	return solver;
}

void solver_destroy ( fluid_solver* solver ){
	if ( solver->u != NULL ){	free ( solver->u );	solver->u = NULL;	}		
	if ( solver->v != NULL ){	free ( solver->v );	solver->v = NULL;	}
	if ( solver->u_prev != NULL){	free ( solver->u_prev );solver->u_prev = NULL; 	}
	if ( solver->v_prev != NULL){	free ( solver->v_prev );solver->v_prev = NULL;	}
	if ( solver->dens != NULL){		free ( solver->dens );	solver->dens = NULL;	}	
	if ( solver->dens_prev != NULL ){	free ( solver->dens_prev );	solver->dens_prev = NULL;	}
	free(solver);
}

void solver_clear_data ( fluid_solver* solver ){
	uint32_t i, size=(solver->N+2)*(solver->N+2);
	for ( i=0 ; i<size ; i++ ) {
		solver->u[i] = solver->v[i] = solver->u_prev[i] = solver->v_prev[i] = solver->dens[i] = solver->dens_prev[i] = 0.0f;
	}
}

void solver_set_initial_velocity ( fluid_solver* solver ){
	uint32_t i, j;
	float grad_to_rad = PI/180.0f;
	for ( i=0 ; i<=solver->N ; i++ ) {
		for ( j=0 ; j<=solver->N ; j++ ) {
			solver->u[IX(i,j)] = ((sin(i * grad_to_rad) + 1))*j*4.0f/(solver->N);
			solver->v[IX(i,j)] = ((cos(i * grad_to_rad) + 1))*i*4.0f/(solver->N);
		}
	}
}

void solver_set_initial_density ( fluid_solver* solver ){
	uint32_t i, j;
	float grad_to_rad = PI/180.0f;
	for ( i=0 ; i<=solver->N ; i++ ) {
		for ( j=0 ; j<=solver->N ; j++ ) {
			solver->dens[IX(i,j)] = ((sin(i * grad_to_rad) + 1) + (cos(j * grad_to_rad) + 1))/4.0f;
		}
	}
}

void solver_add_source ( fluid_solver* solver, float * x, float * s){
	uint32_t i, size=(solver->N+2)*(solver->N+2);
	for ( i=0 ; i<size ; i++ ) x[i] += solver->dt*s[i];
}

void solver_dens_step ( fluid_solver* solver, float * x, float * x0){
	solver_add_source ( solver, x, x0);
	SWAP ( x0, x ); solver_diffuse ( solver, 0, x, x0);
	SWAP ( x0, x ); solver_advect ( solver, 0, x, x0, solver->u, solver->v);
}

void solver_vel_step ( fluid_solver* solver, float * u0, float * v0){
	solver_add_source ( solver, solver->u, u0); solver_add_source ( solver, solver->v, v0);
	SWAP ( u0, solver->u ); solver_diffuse ( solver, 1, solver->u, u0);
	SWAP ( v0, solver->v ); solver_diffuse ( solver, 2, solver->v, v0);
	solver_project ( solver, u0, v0 );
	SWAP ( u0, solver->u ); SWAP ( v0, solver->v );
	solver_advect ( solver, 1, solver->u, u0, u0, v0); solver_advect ( solver, 2, solver->v, v0, u0, v0);
	solver_project ( solver, u0, v0 );
}

void solver_diffuse ( fluid_solver* solver, uint32_t b, float * x, float * x0){
	float a=solver->dt*solver->diff*solver->N*solver->N;
	solver_lin_solve ( solver, b, x, x0, a, 1+4*a );
}

void solver_advect ( fluid_solver* solver, uint32_t b, float * d, float * d0, float * u, float * v){
	uint32_t i, j, i0, j0, i1, j1;
	float x, y, s0, t0, s1, t1, dt0;
	dt0 = solver->dt*solver->N;
	FOR_EACH_CELL
		x = i-dt0*u[IX(i,j)]; y = j-dt0*v[IX(i,j)];
		if (x<0.5f) x=0.5f; if (x>solver->N+0.5f) x=solver->N+0.5f; i0=(int)x; i1=i0+1;
		if (y<0.5f) y=0.5f; if (y>solver->N+0.5f) y=solver->N+0.5f; j0=(int)y; j1=j0+1;
		s1 = x-i0; s0 = 1-s1; t1 = y-j0; t0 = 1-t1;
		d[IX(i,j)] = s0*(t0*d0[IX(i0,j0)]+t1*d0[IX(i0,j1)])+
					 s1*(t0*d0[IX(i1,j0)]+t1*d0[IX(i1,j1)]);
	END_FOR
	solver_set_bnd ( solver, b, d );
}

/* SIMD IMPLEMENTATION */
// #define FOR_EACH_CELL for ( i=1 ; i<=solver->N ; i++ ) { for ( j=1 ; j<=solver->N ; j++ ) {

void solver_lin_solve ( fluid_solver* solver, uint32_t b, float * x, float * x0, float a, float c ){
	uint32_t i, j, k;
	for ( k=0 ; k<20 ; k++ ) {
		FOR_EACH_CELL
			x[IX(i,j)] = (x0[IX(i,j)] + a*(x[IX(i-1,j)]+x[IX(i+1,j)]+x[IX(i,j-1)]+x[IX(i,j+1)]))/c;
		END_FOR
		solver_set_bnd ( solver, b, x );
	}
}

void solver_set_bnd ( fluid_solver* solver, uint32_t b, float * x ){
	uint32_t i;
	uint32_t N = solver->N;
	for ( i=1 ; i<=N ; i++ ) {
		x[IX(0  ,i)] = b==1 ? -x[IX(1,i)] : x[IX(1,i)];
		x[IX(N+1,i)] = b==1 ? -x[IX(N,i)] : x[IX(N,i)];
		x[IX(i,0  )] = b==2 ? -x[IX(i,1)] : x[IX(i,1)];
		x[IX(i,N+1)] = b==2 ? -x[IX(i,N)] : x[IX(i,N)];
	}
	x[IX(0  ,0  )] = 0.5f*(x[IX(1,0  )]+x[IX(0  ,1)]);
	x[IX(0  ,N+1)] = 0.5f*(x[IX(1,N+1)]+x[IX(0  ,N)]);
	x[IX(N+1,0  )] = 0.5f*(x[IX(N,0  )]+x[IX(N+1,1)]);
	x[IX(N+1,N+1)] = 0.5f*(x[IX(N,N+1)]+x[IX(N+1,N)]);
}

/*
void solver_project ( fluid_solver* solver, float * p, float * div ){
	uint32_t i, j;
	FOR_EACH_CELL
		div[IX(i,j)] = -0.5f*(solver->u[IX(i+1,j)]-solver->u[IX(i-1,j)]+solver->v[IX(i,j+1)]-solver->v[IX(i,j-1)])/solver->N;
		p[IX(i,j)] = 0;
	END_FOR	
	solver_set_bnd ( solver, 0, div ); solver_set_bnd ( solver, 0, p );
	solver_lin_solve ( solver, 0, p, div, 1, 4 );
	FOR_EACH_CELL
		solver->u[IX(i,j)] -= 0.5f*solver->N*(p[IX(i+1,j)]-p[IX(i-1,j)]);
		solver->v[IX(i,j)] -= 0.5f*solver->N*(p[IX(i,j+1)]-p[IX(i,j-1)]);
	END_FOR
	solver_set_bnd ( solver, 1, solver->u ); solver_set_bnd ( solver, 2, solver->v );
}*/