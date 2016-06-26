// FluidSimulation.cpp : Defines the entry point for the console application.
//

#include "Particle.h"
#include "RigidBody.h"
#include "Box.h"
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
extern void simulation_step(vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<RigidBody*> rigidBodies, vector<IForce*> forces, float dt, int method);

/* global variables */
static int N;
static float dt, d;
static int dsim;
static int dump_frames;
static int frame_number;

// static Particle *pList;
static vector<Particle*> pVector;
static vector<RigidBody*> rigidBodies;
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
static bool enableBodies = false;
static bool enableHair = false;
static bool enableDrawParticles = true;
static bool enableRenderFluid = false;

/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data(void)
{
	pVector.clear();
	forces.clear();
	rigidBodies.clear();
	if (mouseForce != NULL)
		mouseForce->newMousePosition(Vec2f(0, 0));
}

static void clear_data(void)
{
	int ii, size = pVector.size();

	for (ii = 0; ii<size; ii++) {
		pVector[ii]->reset();
	}
}

static void init_system(void)
{
	const float dist = 0.04;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);

	const float radius = dist * 1.15f;

	// Initialise fluid
	for (int i = 0; i < 1 / dist; i++)
	{
		for (int j = 0; j < 1.25 / dist; j++)
		{
			//if(j == 5)
			//	pVector.push_back(new Particle(Vec2f(-0.95 + ((j % 2) * 0.02) + dist * i, 0.25 - dist * j), 0.4f, radius, true));
			//else
			pVector.push_back(new Particle(Vec2f(-0.95 + ((j % 2) * 0.02) + dist * i, 0.25 - dist * j), 0.4f, radius));
			//	forces.push_back(new GravityForce(pVector[pVector.size() - 1]));
			//	forces.push_back(new DragForce(pVector[pVector.size() - 1]));
			forces.push_back(new WallForce(pVector[pVector.size() - 1]));
		}
	}


	fluidContainer = new FluidContainer(radius, 2.0f, 2.0f);

	//rigidBodies.push_back(new Box(Vec2f(0.4, -0.3), 1, 0.45, 0.15));

	//rigidBodies.push_back(new Box(Vec2f(0.2, -0.8), 1, 0.2, 0.2));

	// Calculate rigid body ghost particles for coupling
	for (int i = 0; i < rigidBodies.size(); i++)
	{
		rigidBodies[i]->generateGhostParticles();

		// Add ghost particles to fluid simulation for rigidbody to fluid interaction
		for (int j = 0; j < rigidBodies[i]->m_GhostParticles.size(); j++)
		{
			pVector.push_back(rigidBodies[i]->m_GhostParticles[j]);
		}
	}

	std::cout << "particles: " << pVector.size() << std::endl;

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
		mouseForce = new MouseForce(vector<Particle*>(), Vec2f(0, 0), dist, 1.0, 1.0);
		forces.push_back(mouseForce);
	}

	//Add wall forces
	//forces.push_back(new WallForce(pVector[0]));
	//forces.push_back(new WallForce(pVector[1]));
	//forces.push_back(new WallForce(pVector[2]));

}

static void rigid_bodies(void)
{
	const float dist = 0.12;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);

	rigidBodies.push_back(new Box(center, 3, 0.5, 0.5));
	rigidBodies.push_back(new Box(Vec2f(0.35, -0.4), 3, 0.25, 0.25));
	/*rigidBodies.push_back(new Box(Vec2f(0.35, -0.54), 1, 0.25, 0.25));
	rigidBodies.push_back(new Box(Vec2f(-0.38, -0.34), 1, 0.25, 0.25));
	rigidBodies.push_back(new Box(Vec2f(0.33, 0.44), 1, 0.25, 0.25));
	rigidBodies.push_back(new Box(Vec2f(-0.36, 0.63), 1, 0.25, 0.25));
	rigidBodies.push_back(new Box(Vec2f(-0.31, 0.25), 1, 0.25, 0.25));
	rigidBodies.push_back(new Box(Vec2f(-0.37, -0.81), 1, 0.25, 0.25));
	rigidBodies.push_back(new Box(Vec2f(-0.8, -0.78), 1, 0.25, 0.25));*/

	rigidBodies.push_back(new Box(Vec2f(0.0, -1.5), 1, 3, 1, true));
	rigidBodies.push_back(new Box(Vec2f(0.0, 1.5), 1, 3, 1, true));
	rigidBodies.push_back(new Box(Vec2f(1.5, 0.0), 1, 1, 3, true));
	rigidBodies.push_back(new Box(Vec2f(-1.5, 0.0), 1, 1, 3, true));
	fluidContainer = new FluidContainer(dist / 2, 2.0f, 2.0f);

	forces.push_back(new DragForce(rigidBodies[0]));
	forces.push_back(new DragForce(rigidBodies[1]));
	forces.push_back(new GravityForce(rigidBodies[0]));
	//forces.push_back(new GravityForce(rigidBodies[1]));
	/*forces.push_back(new GravityForce(rigidBodies[2]));
	forces.push_back(new GravityForce(rigidBodies[3]));
	forces.push_back(new GravityForce(rigidBodies[4]));
	forces.push_back(new GravityForce(rigidBodies[5]));
	forces.push_back(new GravityForce(rigidBodies[6]));*/


	std::cout << "particles: " << pVector.size();
	if (enableMouse)
	{
		mouseForce = new MouseForce(vector<Particle*>(), Vec2f(0, 0), dist, 1.0, 1.0);
		forces.push_back(mouseForce);
	}

}

/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/

static void pre_display(void)
{
	glViewport(0, 0, win_x, win_y);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(-1.0, 1.0, -1.0, 1.0);
	glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
}

static void post_display(void)
{
	frame_number++;

	glutSwapBuffers();
}

static void draw_particles(void)
{
	int size = pVector.size();

	for (int ii = 0; ii< size; ii++)
	{

		pVector[ii]->draw(enableRenderFluid);
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


static void draw_rigidBodies(void)
{
	int size = rigidBodies.size();

	for (int ii = 0; ii < size; ii++)
	{
		rigidBodies[ii]->draw();
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

static void draw_forces(void)
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

static void get_from_UI()
{
	float i, j;
	// int size, flag;
	int hi, hj;
	// float x, y;
	if (!mouse_down[0] && !mouse_down[2] && !mouse_release[0]
		&& !mouse_shiftclick[0] && !mouse_shiftclick[2]) return;

	i = (float)(1 - ((win_x - mx) / ((float)win_x / 2.0)));
	j = (float)(1 - ((my) / ((float)win_y / 2.0)));

	//if ( i<1 || i>N || j<1 || j>N ) return;

	if (mouse_down[0] && enableMouse)
	{
		if (!mouseForce->leftMouseDown)
		{
			vector<Particle*> closeParticles = vector<Particle*>();
			float maxDist = 0.2;
			float dist = 0;

			Vec2f mouseDistance = Vec2f(0, 0);
			Vec2f mousePosition = Vec2f(i, j);
			for (int ii = 0; ii < pVector.size(); ii++)
			{
				mouseDistance = pVector[ii]->m_Position - mousePosition;
				dist = sqrt(mouseDistance[0] * mouseDistance[0] + mouseDistance[1] * mouseDistance[1]);

				if (dist <= maxDist)
					closeParticles.push_back(pVector[ii]);
			}
			mouseForce->selectParticles(closeParticles);
		}
		mouseForce->newMousePosition(Vec2f(i, j));
	}
	else if (enableMouse) {
		mouseForce->clearParticles();
	}


	if (mouse_down[2] && enableMouse)
	{
		if (!mouseForce->rightMouseDown)
		{
			vector<RigidBody*> closeRigidbodies = vector<RigidBody*>();
			float maxDist = 0.1;
			float dist = 0;

			Vec2f mouseDistance = Vec2f(0, 0);
			Vec2f mousePosition = Vec2f(i, j);
			for (int ii = 0; ii < rigidBodies.size(); ii++)
			{
				mouseDistance = rigidBodies[ii]->m_Position - mousePosition;
				dist = sqrt(mouseDistance[0] * mouseDistance[0] + mouseDistance[1] * mouseDistance[1]);

				if (dist <= maxDist)
					closeRigidbodies.push_back(rigidBodies[ii]);
			}
			mouseForce->selectRigidbodies(closeRigidbodies);
		}
		mouseForce->newMousePosition(Vec2f(i, j));
	}
	else if (enableMouse)
	{
		mouseForce->clearRigidbodies();
	}

	if (mouse_down[2]) {
	}

	hi = (int)((hmx / (float)win_x)*N);
	hj = (int)(((win_y - hmy) / (float)win_y)*N);

	if (mouse_release[0]) {
	}

	omx = mx;
	omy = my;
}

static void remap_GUI()
{
	int ii, size = pVector.size();
	for (ii = 0; ii<size; ii++)
	{
		pVector[ii]->reset();
		//pVector[ii]->m_Position[0] = pVector[ii]->m_ConstructPos[0];
		//pVector[ii]->m_Position[1] = pVector[ii]->m_ConstructPos[1];
	}
	size = rigidBodies.size();
	for (ii = 0; ii<size; ii++)
	{
		rigidBodies[ii]->reset();
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
	else if (enableBodies)
		rigid_bodies();
}

/*
----------------------------------------------------------------------
GLUT callback routines
----------------------------------------------------------------------
*/

static void key_func(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'c':
	case 'C':
		clear_data();
		break;

	case 'd':
	case 'D':
		dump_frames = !dump_frames;
		break;

	case 'q':
	case 'Q':
		free_data();
		exit(0);
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
		enableRenderFluid = !enableRenderFluid;
		break;
	case '5':
		enableMouse = !enableMouse;
		break;
	case '6':
		enableStandard = !enableStandard;
		if (enableStandard)
		{
			enableBodies = false;
		}
		free_data();
		load_Scene();
		break;
	case '7':
		enableBodies = !enableBodies;
		if (enableBodies)
		{
			enableStandard = false;
		}
		free_data();
		load_Scene();
		break;
	case '9':
		enableDrawParticles = !enableDrawParticles;
		break;
	}
}

static void mouse_func(int button, int state, int x, int y)
{
	omx = mx = x;
	omx = my = y;

	if (!mouse_down[0]) { hmx = x; hmy = y; }
	if (mouse_down[button]) mouse_release[button] = state == GLUT_UP;
	if (mouse_down[button]) mouse_shiftclick[button] = glutGetModifiers() == GLUT_ACTIVE_SHIFT;
	mouse_down[button] = state == GLUT_DOWN;
}

static void motion_func(int x, int y)
{
	mx = x;
	my = y;
}

static void reshape_func(int width, int height)
{
	glutSetWindow(win_id);
	glutReshapeWindow(width, height);

	win_x = width;
	win_y = height;
}

static void idle_func(void)
{
	if (dsim)
		simulation_step(pVector, fluidContainer, rigidBodies, forces, dt, method);
	else
	{
		remap_GUI();
	}

	get_from_UI();
	glutSetWindow(win_id);
	glutPostRedisplay();
}

static void display_func(void)
{
	pre_display();

	draw_forces();

	if (enableDrawParticles)
	{
		draw_particles();
		draw_rigidBodies();
	}

	post_display();
}


/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window(void)
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);

	glutInitWindowPosition(0, 0);
	glutInitWindowSize(win_x, win_y);
	win_id = glutCreateWindow("Fluid Simulator");

	glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glutSwapBuffers();
	glClear(GL_COLOR_BUFFER_BIT);
	glutSwapBuffers();

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glEnable(GL_BLEND);
	glEnable(GL_POINT_SMOOTH);
	glHint(GL_POINT_SMOOTH_HINT, GL_FASTEST);

	pre_display();

	glutKeyboardFunc(key_func);
	glutMouseFunc(mouse_func);
	glutMotionFunc(motion_func);
	glutReshapeFunc(reshape_func);
	glutIdleFunc(idle_func);
	glutDisplayFunc(display_func);
}


/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/

int main(int argc, char ** argv)
{
	glutInit(&argc, argv);

	if (argc == 1) {
		N = 64;
		dt = 0.01;
		d = 5.f;
		fprintf(stderr, "Using defaults : N=%d dt=%g d=%g\n",
			N, dt, d);
	}
	else {
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		d = atof(argv[3]);
	}

	printf("\n\nHow to use this application:\n\n");
	printf("\t Toggle construction/simulation display with the spacebar key\n");
	printf("\t Use the '1-9' keys to enable/disable the following functions : \n");
	printf("\t 1 : Eulers method \n");
	printf("\t 2 : Midpoint \n");
	printf("\t 3 : RungKutta \n");
	printf("\t 5 : Mouse interaction \n");
	printf("\t 6 : Standard display \n");
	printf("\t 7 : Rigid Body display \n");
	printf("\t 9 : Draw particles \n");
	printf("\t Dump frames by pressing the 'd' key\n");
	printf("\t Quit by pressing the 'q' key\n");

	load_Scene();

	win_x = 512;
	win_y = 512;
	open_glut_window();

	glutMainLoop();

	exit(0);
}