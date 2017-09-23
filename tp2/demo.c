#include "solver.h"


/* global variables */

fluid_solver* solver;
fluid_app*	app;



/*
  ----------------------------------------------------------------------
  Funciones de OpenGL para dibujar los contenidos de las matrices
  ----------------------------------------------------------------------
*/

void pre_display ()
{
	glViewport ( 0, 0, app->win_x, app->win_y );
	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	gluOrtho2D ( 0.0, 1.0, 0.0, 1.0 );
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
}

void post_display ( void )
{
	glutSwapBuffers ();
}

void draw_velocity ()
{
	uint32_t i, j;
	float x, y, h;

	h = 1.0f/solver->N;

	glColor3f ( 1.0f, 1.0f, 1.0f );
	glLineWidth ( 1.0f );

	glBegin ( GL_LINES );

		for ( i=1 ; i<=solver->N ; i++ ) {
			x = (i-0.5f)*h;
			for ( j=1 ; j<=solver->N ; j++ ) {
				y = (j-0.5f)*h;

				glVertex2f ( x, y );
				glVertex2f ( x+solver->u[IX(i,j)], y+solver->v[IX(i,j)] );
			}
		}

	glEnd ();
}

void draw_density ()
{
	uint32_t i, j;
	float x, y, h, d00, d01, d10, d11;

	h = 1.0f/solver->N;

	if(app->app_mode != APP_CLIENT)
		app->mask_value = (ADD_R(app->mask_value, 2)|ADD_G(app->mask_value, 3)|ADD_B(app->mask_value, 1));

	glBegin ( GL_QUADS );

		for ( i=0 ; i<=solver->N ; i++ ) {
			x = (i-0.5f)*h;
			for ( j=0 ; j<=solver->N ; j++ ) {
				y = (j-0.5f)*h;

				d00 = solver->dens[IX(i,j)];
				d01 = solver->dens[IX(i,j+1)];
				d10 = solver->dens[IX(i+1,j)];
				d11 = solver->dens[IX(i+1,j+1)];

				glColor3f ( d00 * GET_R(app->mask_value), d00 * GET_G(app->mask_value), d00 * GET_B(app->mask_value)); glVertex2f ( x, y );
				glColor3f ( d10 * GET_R(app->mask_value), d10 * GET_G(app->mask_value), d10 * GET_B(app->mask_value)); glVertex2f ( x+h, y );
				glColor3f ( d11 * GET_R(app->mask_value), d11 * GET_G(app->mask_value), d11 * GET_B(app->mask_value)); glVertex2f ( x+h, y+h );
				glColor3f ( d01 * GET_R(app->mask_value), d01 * GET_G(app->mask_value), d01 * GET_B(app->mask_value)); glVertex2f ( x, y+h );
			}
		}

	glEnd ();
}

/*
  ----------------------------------------------------------------------
   Funciones de OpenGL para levantar las acciones del mouse
  ----------------------------------------------------------------------
*/

void get_from_UI (float * d, float * u, float * v )
{
	uint32_t i, j, size = (solver->N+2)*(solver->N+2);

	for ( i=0 ; i<size ; i++ ) {
		u[i] = v[i] = d[i] = 0.0f;
	}

	if ( !app->mouse_down[0] && !app->mouse_down[2] ) return;

	i = (int)((       app->mx /(float)app->win_x)*solver->N+1);
	j = (int)(((app->win_y-app->my)/(float)app->win_y)*solver->N+1);

	if ( i<1 || i>solver->N || j<1 || j>solver->N ) return;

	if ( app->mouse_down[0] ) {
		u[IX(i,j)] = app->force * (app->mx-app->omx);
		v[IX(i,j)] = app->force * (app->omy-app->my);
	}

	if ( app->mouse_down[2] ) {
		d[IX(i,j)] = app->source;
	}

	app->omx = app->mx;
	app->omy = app->my;

	return;
}

/*
  ----------------------------------------------------------------------
   Funciones de GLUT para levantar las acciones de teclado
  ----------------------------------------------------------------------
*/

void key_func (unsigned char key, int x, int y )
{
	switch ( key )
	{
		case 'c':
		case 'C':
			solver_clear_data (solver);
			break;

		case 'q':
		case 'Q':
			solver_destroy (solver);
			free(app);
			exit ( 0 );
			break;

		case 'v':
		case 'V':
			if(app->app_mode == APP_INTERACTIVE)
				app->dvel = !app->dvel;
			break;
	}
}

void mouse_func (int button, int state, int x, int y )
{
	if(app->app_mode != APP_INTERACTIVE && app->app_mode != APP_CLIENT)
		return;
	app->omx = app->mx = x;
	app->omx = app->my = y;

	app->mouse_down[button] = state == GLUT_DOWN;
}

void motion_func ( int x, int y )
{
	app->mx = x;
	app->my = y;
}

void reshape_func (int width, int height )
{
	glutSetWindow ( app->win_id );
	glutReshapeWindow ( width, height );

	app->win_x = width;
	app->win_y = height;
}

void idle_func ()
{
	get_from_UI ( solver->dens_prev, solver->u_prev, solver->v_prev );

	solver_vel_step ( solver, solver->u_prev, solver->v_prev);
	solver_dens_step ( solver, solver->dens, solver->dens_prev);

	glutSetWindow ( app->win_id );
	glutPostRedisplay ();
}

void display_func ( void )
{
	pre_display ();

		if ( app->dvel ) draw_velocity ();
		else		draw_density ();

	post_display ();
}


/*
  ----------------------------------------------------------------------
   Inicialización de la ventana de GLUT
  ----------------------------------------------------------------------
*/

void open_glut_window ( void )
{
	glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE );

	glutInitWindowPosition ( 0, 0 );
	glutInitWindowSize ( app->win_x, app->win_y );
	app->win_id = glutCreateWindow ( "Orga2 | 2017 - c2 | TP2" );

	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();

	pre_display ();

	glutKeyboardFunc ( key_func );
	glutMouseFunc ( mouse_func );
	glutMotionFunc ( motion_func );
	glutReshapeFunc ( reshape_func );
	glutIdleFunc ( idle_func );
	glutDisplayFunc ( display_func );
}


/*
  ----------------------------------------------------------------------
   Thread que lee los valores enviados por el server (Reservado para la entrega)
  ----------------------------------------------------------------------

*/

/* this function is run by the second thread */
void *read_mask(void *void_ptr){
	int sock, status, buflen;
	unsigned sinlen;
	uint32_t buffer[MAXBUF/sizeof(uint32_t)];
	struct sockaddr_in sock_in;
	int yes = 1;

	sinlen = sizeof(struct sockaddr_in);
	memset(&sock_in, 0, sinlen);

	sock = socket (PF_INET, SOCK_DGRAM, IPPROTO_UDP);

	sock_in.sin_addr.s_addr = htonl(INADDR_ANY);
	sock_in.sin_port = htons(app->client_PORT);
	sock_in.sin_family = PF_INET;

	status = bind(sock, (struct sockaddr *)&sock_in, sinlen);
	printf("Bind Status = %d\n", status);

	status = getsockname(sock, (struct sockaddr *)&sock_in, &sinlen);
	printf("Sock port %d\n",htons(sock_in.sin_port));

	buflen = MAXBUF;
	memset(buffer, 0, buflen);
	while(!app->should_exit){
		status = recvfrom(sock, buffer, buflen, 0, (struct sockaddr *)&sock_in, &sinlen);
		printf("sendto Status = %d\n", status);
		uint32_t size = status / sizeof(uint32_t);
		if(size > app->client_ID){
			app->mask_value = buffer[app->client_ID];	
		}
		fflush(stdout);
		usleep(10000);
	}
	shutdown(sock, 2);
	close(sock);
	return NULL;

}


/*
  ----------------------------------------------------------------------
   MAIN - Entrada al programa
  ----------------------------------------------------------------------
*/




int main ( int argc, char ** argv ){
	app = malloc(sizeof(fluid_app));
	app->should_exit = false;
	app->dvel = 0;
	app->force = 20.0f;
	app->source = 100.0f;
	app->win_x = 1028;
	app->win_y = 768;
	app->mask_value = 0xFF0F0F;

	if(argc > 4){
		app->should_exit = true;
	}else if(argc > 1){
		if(strcmp(argv[1], "--help")== 0){
			app->should_exit = true;
		}else{
			int check_app_mode = atoi(argv[1]);
			if(check_app_mode >= 0 && check_app_mode <= 4){
				app->app_mode = check_app_mode;
				if(app->app_mode == APP_CLIENT){
					if(argc != 4){
						app->should_exit = true;
					}else{
						app->client_ID = atoi(argv[2]);
						app->client_PORT = atoi(argv[3]);
					}
				}
			}else{
				app->should_exit = true;
			}
		}
	}else{
		app->app_mode = APP_DEMO;
	}

	if (app->should_exit) {
		fprintf ( stderr, "Uso de la aplicacion:\n");
		fprintf ( stderr, "demo m [ID PORT] donde m es el modo de ejecucion y ID PORT (opcional) el Id:puerto de cliente\n" );\
		fprintf ( stderr, "0 para correr una Demo automatica\n" );
		fprintf ( stderr, "1 para correr una Demo interactiva\n" );
		fprintf ( stderr, "\t(click izquierdo modifica el campo vectorial de fuerzas,\n" );
		fprintf ( stderr, "\t click derecho agrega densidad, v muestra el campo de fuerzas\n" );
		fprintf ( stderr, "\t y q finaliza la ejeucion)\n" );
		fprintf ( stderr, "2 para correr los tests\n" );
		fprintf ( stderr, "3 ID PORT para correr en modo cliente-servidor\n" );
		fprintf ( stderr, "\t ID es el numero de maquina\n" );		
		fprintf ( stderr, "\t PORT es el puerto por el que recibe los mensajes\n" );		
		free(app);
		exit ( 1 );
	}



	if(app->app_mode != APP_TEST){
		//	N = 256; 	dt = 0.05;	diff = 0.0f;	visc = 0.0f;	force = 20.0f;	source = 600.0f;
		solver = solver_create(128, 0.05, 0, 0);

		if(app->app_mode != APP_INTERACTIVE){
			solver_set_initial_density(solver);
			solver_set_initial_velocity(solver);
		}
		pthread_t read_thread;
		if(app->app_mode == APP_CLIENT){
			if(pthread_create(&read_thread, NULL, read_mask, NULL)) {
				fprintf(stderr, "Error creating thread\n");
			}
		}
		glutInit ( &argc, argv );
		open_glut_window ();

		glutMainLoop ();
		app->should_exit = true;
		if(app->app_mode == APP_CLIENT){
			if(pthread_join(read_thread, NULL)) {
				fprintf(stderr, "Error joining thread\n");
			}
		}	
		solver_destroy(solver);

	}else{
		int sizes[] = {16, 32, 64, 128, 256, 512};
		char density_file_name[256], density_diff_name[256], density_mine_name[256], velocity_u_file_name[256], velocity_u_diff_name[256], velocity_u_mine_name[256], velocity_v_file_name[256], velocity_v_diff_name[256], velocity_v_mine_name[256];
		for(uint32_t i = 0; i < (sizeof(sizes) / sizeof(int)); i++){
			int size = sizes[i];
			sprintf(density_file_name, "tmp/catedra_matrix_dens_%d.bmp", size);
			sprintf(density_diff_name, "tmp/matrix_dens_%d_diff.bmp", size);
			sprintf(density_mine_name, "tmp/matrix_dens_%d_mine.bmp", size);
			sprintf(velocity_u_file_name, "tmp/catedra_matrix_vel_u_%d.bmp", size);
			sprintf(velocity_u_diff_name, "tmp/matrix_vel_u_%d_diff.bmp", size);
			sprintf(velocity_u_mine_name, "tmp/matrix_vel_u_%d_mine.bmp", size);
			sprintf(velocity_v_file_name, "tmp/catedra_matrix_vel_v_%d.bmp", size);
			sprintf(velocity_v_diff_name, "tmp/matrix_vel_v_%d_diff.bmp", size);
			sprintf(velocity_v_mine_name, "tmp/matrix_vel_v_%d_mine.bmp", size);
			//	N = 256; 	dt = 0.05;	diff = 0.0f;	visc = 0.0f;	force = 20.0f;	source = 600.0f;
			solver = solver_create(size, 0.05, 0, 0);
			solver_set_initial_density(solver);
			solver_set_initial_velocity(solver);
			int k = 0;
			int RUNS = 20; 
			for(k =0 ; k < RUNS; k++){
				solver_vel_step ( solver, solver->u_prev, solver->v_prev);
				solver_dens_step ( solver, solver->dens, solver->dens_prev);		
			}
			draw_alpha(size, solver->dens, density_mine_name);
			draw_diff(size, solver->dens, density_file_name, density_diff_name);
			draw_alpha(size, solver->u, velocity_u_mine_name);
			draw_diff(size, solver->u, velocity_u_file_name, velocity_u_diff_name);
			draw_alpha(size, solver->v, velocity_v_mine_name);
			draw_diff(size, solver->v, velocity_v_file_name, velocity_v_diff_name);
			solver_destroy(solver);
		}
	}
	
	free(app);

	exit ( 0 );
}
