// TinkerToy.cpp : Defines the entry point for the console application.
//

#include "Particle.h"
#include "IForce.h"
#include "MouseForce.h"
#include "DragForce.h"
#include "GravityForce.h"
#include "FluidContainer.h"
#include "WallForce.h"

#include <vector>
#include <stdlib.h>
#include <stdio.h>
//#include <GL/glut.h>
#include "GLUtil.h"

using namespace std;

/* macros */


/* external definitions (from solver) */
extern void simulation_step(vector<Particle*> pVector, FluidContainer* fluidContainer, vector<IForce*> forces, float dt, int method );

/* global variables */
static int N;
static float dt, d;
static int dsim;
static int dump_frames;
static int frame_number;

// static Particle *pList;
static vector<Particle*> pVector;
static FluidContainer* fluidContainer;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;

static vector<IForce*> forces = vector<IForce*>();
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
	const float dist = 0.15;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);

	for (int i = 0; i < 2 / dist; i++)
	{
		for (int j = 0; j < 1 / dist; j++)
		{
			pVector.push_back(new Particle(Vec2f(-1 + dist * i, 0 - dist * j), 0.2f, dist / 2));
			forces.push_back(new GravityForce(pVector[pVector.size() - 1]));
			forces.push_back(new WallForce(pVector[pVector.size() - 1]));
		}
	}

	fluidContainer = new FluidContainer(dist / 2, 2.0f, 2.0f);

	std::cout << "particles: " << pVector.size();
	
	//pVector.push_back(new Particle(center + offset + offset, 1));
	//pVector.push_back(new Particle(center + offset + offset + offset, 1));


	// Add gravity
	//forces.push_back(new GravityForce(pVector[0]));
	//forces.push_back(new GravityForce(pVector[1]));
	//forces.push_back(new GravityForce(pVector[2]));

	// Add drag
	//forces.push_back(new DragForce(pVector[0]));
	//forces.push_back(new DragForce(pVector[1]));
	//forces.push_back(new DragForce(pVector[2]));

	//Add spring forces
	//forces.push_back(new SpringForce(pVector[0], pVector[1], dist, 1.0, 1.0));
	//forces.push_back(new SpringForce(pVector[0], pVector[2], dist, 1.0, 0.5));
	
	//Add Constraints
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
	glClearColor ( 0.2f, 0.2f, 0.2f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
}

static void post_display ( void )
{
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

	//glEnableClientState(GL_VERTEX_ARRAY);
	//glEnableClientState(GL_COLOR_ARRAY);

	//glPointSize(2.5f*kParticleRadius*kScreenWidth / kViewWidth);

	//glColorPointer(4, GL_FLOAT, sizeof(Rgba), shadedParticleColours);
	//glVertexPointer(2, GL_FLOAT, sizeof(Particle), particles);
	//glDrawArrays(GL_POINTS, 0, particleCount);

	//glDisableClientState(GL_COLOR_ARRAY);
	//glDisableClientState(GL_VERTEX_ARRAY);

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
		simulation_step(pVector, fluidContainer, forces, dt, method);
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
	win_id = glutCreateWindow ( "Fluid Simulator" );

	glClearColor ( 0.2f, 0.2f, 0.2f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glEnable(GL_BLEND);
	glEnable(GL_POINT_SMOOTH);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

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
		dt = 0.05f;
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

