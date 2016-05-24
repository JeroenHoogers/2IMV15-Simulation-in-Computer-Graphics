// TinkerToy.cpp : Defines the entry point for the console application.
//

#include "Particle.h"
#include "IForce.h"
#include "SpringForce.h"
#include "AngularForce.h"
#include "MouseForce.h"
#include "RodConstraint.h"
#include "DragForce.h"
#include "CircularWireConstraint.h"
#include "WireConstraint.h"
#include "PointConstraint.h"
#include "GravityForce.h"
#include "WallForce.h"
#include "imageio.h"

#include <vector>
#include <stdlib.h>
#include <stdio.h>
//#include <GL/glut.h>
#include "GLUtil.h"

using namespace std;

/* macros */


/* external definitions (from solver) */
extern void simulation_step(vector<Particle*> pVector, vector<IForce*> forces, vector<IConstraint*> constraints, float dt, int method );

/* global variables */
static int N;
static float dt, d;
static int dsim;
static int dump_frames;
static int frame_number;

// static Particle *pList;
static vector<Particle*> pVector;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;

static vector<IForce*> forces = vector<IForce*>();
static vector<IConstraint*> constraints = vector<IConstraint*>();
static MouseForce* mouseForce = NULL;

static int method = 0;
static bool enableMouse = true;
static bool enableStandard = true;
static bool enableCloth = false;
static bool enableHair = false;
static bool enableDrawParticles = true;

/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data ( void )
{
	pVector.clear();
	forces.clear();
	constraints.clear();
	if (mouseForce != NULL)
		mouseForce->newMousePosition(Vec2f(0, 0));
}

static void clear_data ( void )
{
	int ii, size = pVector.size();

	for(ii=0; ii<size; ii++){
		pVector[ii]->reset();
	}
}

static void init_system(void)
{
	const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);

	// Create three particles
	pVector.push_back(new Particle(center + offset, 1));
	pVector.push_back(new Particle(center + offset + offset, 1));
	pVector.push_back(new Particle(center + offset + offset + offset, 1));

	// Add gravity
	forces.push_back(new GravityForce(pVector[0]));
	forces.push_back(new GravityForce(pVector[1]));
	forces.push_back(new GravityForce(pVector[2]));

	// Add drag
	forces.push_back(new DragForce(pVector[0]));
	forces.push_back(new DragForce(pVector[1]));
	forces.push_back(new DragForce(pVector[2]));

	//Add spring forces
	//forces.push_back(new SpringForce(pVector[0], pVector[1], dist, 1.0, 1.0));
	forces.push_back(new SpringForce(pVector[1], pVector[2], dist, 1.0, 1.0));
	//forces.push_back(new SpringForce(pVector[0], pVector[2], dist, 1.0, 0.5));
	
	//Add Constraints
	constraints.push_back(new RodConstraint(pVector[1], pVector[0], dist));
	constraints.push_back(new CircularWireConstraint(pVector[0], center, dist));
	//constraints.push_back(new WireConstraint(pVector[0], pVector[0]->m_ConstructPos, 0));
	//constraints.push_back(new WireConstraint(pVector[0], pVector[0]->m_ConstructPos, 1));
	
	// Add mouse force
	if (enableMouse)
	{
		mouseForce = new MouseForce(pVector[0], Vec2f(0, 0), dist, 1.0, 1.0);
		forces.push_back(mouseForce);
	}

	//Add wall forces
	//forces.push_back(new WallForce(pVector[0]));
	//forces.push_back(new WallForce(pVector[1]));
	//forces.push_back(new WallForce(pVector[2]));
}	

static void createClothGrid()
{
	const double dist = 0.1;
	const int length = 12;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);
	const Vec2f offseth(0.0, dist);
	double mass = 0.5;

	//Initialise grid
	for (int i = -(length/2); i < (length/2); i++)
	{
		for (int j = -(length/2); j < (length/2); j++)
		{
			//add a higher mass to outer particles
			if (i == -(length / 2) || i == (length / 2) - 1 || j == -(length / 2) || j == (length / 2) - 1)
				mass = 1.5;
			else
				mass = 0.5;
			
			//set 2 fixed points for the cloth
			//if ((j == -(length / 2) || j == (length / 2 - 1) || j == 1) && i == (length / 2 - 1))
				//pVector.push_back(new Particle(center + ((float)i * offseth) + ((float)j * offset), mass, true));
			//add remaining points
			//else
			pVector.push_back(new Particle(center + ((float)i * offseth) + ((float)j * offset), mass));

			if ((j == -(length / 2) || j == (length / 2 - 1) || j == 1) && i == (length / 2 - 1))
			{
				//constraints.push_back(new WireConstraint(pVector[pVector.size() - 1], pVector[pVector.size() - 1]->m_ConstructPos, 0));
				constraints.push_back(new WireConstraint(pVector[pVector.size() - 1], pVector[pVector.size() - 1]->m_ConstructPos, 1));
				//constraints.push_back(new PointConstraint(pVector[pVector.size() - 1], pVector[pVector.size() - 1]->m_ConstructPos));
			}
		}
	}

	for (int i = 0; i < pVector.size(); i++)
	{
		// Add gravity
		forces.push_back(new GravityForce(pVector[i]));

		// Add drag
		forces.push_back(new DragForce(pVector[i]));
	}

	//Add spring force
	for (int i = 0; i < pVector.size() - 1; i++)
	{
		//add horizontal springs
		if (!((i + 1) % length == 0))
			forces.push_back(new SpringForce(pVector[i], pVector[i + 1], dist, 0.04, 0.25));
		if (i < pVector.size() - length)
		{
			//add vertical springs
			forces.push_back(new SpringForce(pVector[i], pVector[i + length], dist, 0.04, 0.25));

			//add diagonal springs
			if (!((i + 1) % length == 0))
				forces.push_back(new SpringForce(pVector[i + length], pVector[i + 1], dist * sqrt(2), 0.06, 0.5));
			if (!((i) % length == 0))
				forces.push_back(new SpringForce(pVector[i + length], pVector[i - 1], dist * sqrt(2), 0.06, 0.5));

			//add horizontal damping springs
			if (i < pVector.size() - (2 * length))
			{
				forces.push_back(new SpringForce(pVector[i], pVector[i + 2 * length], 2*dist, 0.02, 0.1));

				//if (pVector[i + 2]->m_isFixed == false)
				//	forces.push_back(new AngularForce(pVector[i], pVector[i + 1], pVector[i + 2], 180, 0.005, 0.05));
			}
		}
		//add vertical damping springs
		if (!((i + 2) % length == 0) && !((i + 1) % length == 0))
		{
			forces.push_back(new SpringForce(pVector[i], pVector[i + 2], 2 * dist, 0.02, 0.1));
			//if (pVector[i + 2]->m_isFixed == false)
			//	forces.push_back(new AngularForce(pVector[i], pVector[i + 1], pVector[i + 2], 180, 0.005, 0.05));
		}
		
	}

	//Add mouse force
	if (enableMouse)
	{
		mouseForce = new MouseForce(pVector[0], Vec2f(0, 0), dist, 1.0, 1.0);
		forces.push_back(mouseForce);
	}

	//Contstraints
	//test : 
	//constraints.push_back(new CircularWireConstraint(pVector[length*length - 1], pVector[length*length - 1]->m_ConstructPos + offseth, dist));
	//constraints.push_back(new CircularWireConstraint(pVector[length*length - length], pVector[length*length - length]->m_ConstructPos + offseth, dist));
	// TODO : LineWireConstraint

	// Add Wall forces
	for (int i = 0; i < pVector.size(); i++)
	{
		forces.push_back(new WallForce(pVector[i]));
	}

	//Angular springs
	//{ }
}

static void createHairDisplay()
{
	const double dist = 0.04;
	const int length = 32;
	const Vec2f center(0.0, 0.35);
	const Vec2f offseth(0.0, dist);
	double mass = 0.1;

	//Initialise grid
	for (int i = -(length / 2); i < (length / 2); i++)
	{
			//set 2 fixed points for the cloth
			if (i == (length / 2 - 1))
				pVector.push_back(new Particle(center + ((float)i * offseth), mass, true));
			//add remaining points
			else
				pVector.push_back(new Particle(center + ((float)i * offseth), mass));
	}

	for (int i = 0; i < pVector.size(); i++)
	{
		// Add gravity
		forces.push_back(new GravityForce(pVector[i]));

		// Add drag
		forces.push_back(new DragForce(pVector[i]));
	}

	//Add spring force
	for (int i = 0; i < pVector.size() - 1; i++)
	{
		//add horizontal springs
		if (!((i + 1) % length == 0))
			forces.push_back(new SpringForce(pVector[i], pVector[i + 1], dist, 0.1, 0.5));
	}

	//Add mouse force
	if (enableMouse)
	{
		mouseForce = new MouseForce(pVector[0], Vec2f(0, 0), dist, 1.0, 1.0);
		forces.push_back(mouseForce);
	}

	//Contstraints
	//{}

	//Add angular springs
	for (int i = 0; i < pVector.size() - 2; i++)
	{
		if (pVector[i + 2]->m_isFixed == false)
		forces.push_back(new AngularForce(pVector[i], pVector[i + 1], pVector[i + 2], 180, 0.005, 0.05));
	}
}

/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/

static void pre_display ( void )
{
	glViewport ( 0, 0, win_x, win_y );
	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	gluOrtho2D ( -1.0, 1.0, -1.0, 1.0 );
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
}

static void post_display ( void )
{
	// Write frames if necessary.
	if (dump_frames) {
		const int FRAME_INTERVAL = 4;
		if ((frame_number % FRAME_INTERVAL) == 0) {
			const unsigned int w = glutGet(GLUT_WINDOW_WIDTH);
			const unsigned int h = glutGet(GLUT_WINDOW_HEIGHT);
			unsigned char * buffer = (unsigned char *) malloc(w * h * 4 * sizeof(unsigned char));
			if (!buffer)
				exit(-1);
			// glRasterPos2i(0, 0);
			glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
			static char filename[80];
			sprintf(filename, "snapshots/img%.5i.png", frame_number / FRAME_INTERVAL);
			printf("Dumped %s.\n", filename);
			saveImageRGBA(filename, buffer, w, h);
			
			free(buffer);
		}
	}
	frame_number++;
	
	glutSwapBuffers ();
}

static void draw_particles ( void )
{
	int size = pVector.size();

	for(int ii=0; ii< size; ii++)
	{
		pVector[ii]->draw();
	}
}

static void draw_forces ( void )
{

	//if (delete_this_dummy_spring)
	//	delete_this_dummy_spring->draw();

	// draw forces
	for (int i = 0; i < forces.size(); i++)
	{
		forces[i]->draw();
	}
}

static void draw_constraints ( void )
{
	// change this to iteration over full set

	//draw all rod constraints
	int size = constraints.size();
	for (int i = 0; i < size; i++)
	{
		constraints[i]->draw();
	}
	/*if (delete_this_dummy_rod)
		delete_this_dummy_rod->draw();
	if (delete_this_dummy_wire)
		delete_this_dummy_wire->draw();*/
}

/*
----------------------------------------------------------------------
relates mouse movements to tinker toy construction
----------------------------------------------------------------------
*/

static void get_from_UI ()
{
	float i, j;
	// int size, flag;
	int hi, hj;
	// float x, y;
	if ( !mouse_down[0] && !mouse_down[2] && !mouse_release[0] 
	&& !mouse_shiftclick[0] && !mouse_shiftclick[2] ) return;

	i = (float)(1 - ((win_x-mx)/((float)win_x / 2.0)));
	j = (float)(1 - ((		my)/((float)win_y / 2.0)));

	//if ( i<1 || i>N || j<1 || j>N ) return;

	if ( mouse_down[0] && enableMouse) {
		if (!mouseForce->selected)
		{
			Vec2f currentdis = Vec2f(2, 2);
			Vec2f newdis = Vec2f(2, 2);
			for (int ii = 0; ii < pVector.size(); ii++)
			{
				
				newdis = pVector[ii]->m_Position - Vec2f(i, j);
				if (abs(newdis[0]) + abs(newdis[1]) < abs(currentdis[0]) + abs(currentdis[1]))
				{
					currentdis = newdis;
					mouseForce->selectParticle(pVector[ii]);
				}
			}
		}
		mouseForce->newMousePosition(Vec2f(i, j));
	}
	else if (enableMouse) {
		mouseForce->clearParticle();
	}

	if ( mouse_down[2] ) {
	}

	hi = (int)((       hmx /(float)win_x)*N);
	hj = (int)(((win_y-hmy)/(float)win_y)*N);

	if( mouse_release[0] ) {
	}

	omx = mx;
	omy = my;
}

static void remap_GUI()
{
	int ii, size = pVector.size();
	for(ii=0; ii<size; ii++)
	{
		pVector[ii]->reset();
		//pVector[ii]->m_Position[0] = pVector[ii]->m_ConstructPos[0];
		//pVector[ii]->m_Position[1] = pVector[ii]->m_ConstructPos[1];
	}
}

static void load_Scene(void)
{
	dsim = 0;
	dump_frames = 0;
	frame_number = 0;

	if (enableStandard)
		init_system();
	else if (enableCloth)
		createClothGrid();
	else if (enableHair)
		createHairDisplay();
}

/*
----------------------------------------------------------------------
GLUT callback routines
----------------------------------------------------------------------
*/

static void key_func ( unsigned char key, int x, int y )
{
	switch ( key )
	{
	case 'c':
	case 'C':
		clear_data ();
		break;

	case 'd':
	case 'D':
		dump_frames = !dump_frames;
		break;

	case 'q':
	case 'Q':
		free_data ();
		exit ( 0 );
		break;

	case ' ':
		dsim = !dsim;
		break;

	case '1':
		method = 0;
		break;
	case '2':
		method = 1;
		break;
	case '3':
		method = 2;
		break;
	case '4':
		break;
	case '5':
		enableMouse = !enableMouse;
		break;
	case '6':
		enableStandard = !enableStandard;
		if (enableStandard) 
		{
			enableCloth = false;
			enableHair = false;
		}
		free_data();
		load_Scene();
		break;
	case '7':
		enableCloth = !enableCloth;
		if (enableCloth)
		{
			enableStandard = false;
			enableHair = false;
		}
		free_data();
		load_Scene();
		break;
	case '8':
		enableHair = !enableHair;
		if (enableHair)
		{
			enableStandard = false;
			enableCloth = false;
		}
		free_data();
		load_Scene();
		break;
	case '9':
		enableDrawParticles = !enableDrawParticles;
		break;
	}
}

static void mouse_func ( int button, int state, int x, int y )
{
	omx = mx = x;
	omx = my = y;

	if(!mouse_down[0]){hmx=x; hmy=y;}
	if(mouse_down[button]) mouse_release[button] = state == GLUT_UP;
	if(mouse_down[button]) mouse_shiftclick[button] = glutGetModifiers()==GLUT_ACTIVE_SHIFT;
	mouse_down[button] = state == GLUT_DOWN;
}

static void motion_func ( int x, int y )
{
	mx = x;
	my = y;
}

static void reshape_func ( int width, int height )
{
	glutSetWindow ( win_id );
	glutReshapeWindow ( width, height );

	win_x = width;
	win_y = height;
}

static void idle_func ( void )
{
	if ( dsim ) 
		simulation_step(pVector, forces, constraints, dt, method);
	else        
	{
		remap_GUI();
	}

	get_from_UI();
	glutSetWindow ( win_id );
	glutPostRedisplay ();
}

static void display_func ( void )
{
	pre_display ();

	draw_forces();
	draw_constraints();

	if (enableDrawParticles)
	{
		draw_particles();
	}

	post_display ();
}


/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window ( void )
{
	glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE );

	glutInitWindowPosition ( 0, 0 );
	glutInitWindowSize ( win_x, win_y );
	win_id = glutCreateWindow ( "Tinkertoys!" );

	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

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
main --- main routine
----------------------------------------------------------------------
*/

int main ( int argc, char ** argv )
{
	glutInit ( &argc, argv );

	if ( argc == 1 ) {
		N = 64;
		dt = 0.1f;
		d = 5.f;
		fprintf ( stderr, "Using defaults : N=%d dt=%g d=%g\n",
			N, dt, d );
	} else {
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		d = atof(argv[3]);
	}

	printf ( "\n\nHow to use this application:\n\n" );
	printf ( "\t Toggle construction/simulation display with the spacebar key\n" );
	printf("\t Use the '1-9' keys to enable/disable the following functions : \n");
	printf("\t 1 : Eulers method \n");
	printf("\t 2 : Midpoint \n");
	printf("\t 3 : RungKutta \n");
	printf("\t 5 : Mouse interaction \n");
	printf("\t 6 : Standard display \n");
	printf("\t 7 : Cloth display \n");
	printf("\t 8 : Hair display \n");
	printf("\t 9 : Draw particles \n");
	printf ( "\t Dump frames by pressing the 'd' key\n" );
	printf ( "\t Quit by pressing the 'q' key\n" );
	
	load_Scene();

	win_x = 512;
	win_y = 512;
	open_glut_window ();

	glutMainLoop ();

	exit ( 0 );
}

