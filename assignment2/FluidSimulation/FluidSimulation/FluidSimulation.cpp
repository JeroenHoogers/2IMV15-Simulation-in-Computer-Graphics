// FluidSimulation.cpp : Defines the entry point for the console application.
//

#include "Particle.h"
#include "RigidBody.h"
#include "Box.h"
#include "IForce.h"
#include "MouseForce.h"
#include "SpringForce.h"
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
static bool enableMoreBodies = false;
static bool enableCloth = false;
static bool enableDrawParticles = true;
static bool enableRenderFluid = false;
static bool enableDrawGrid = true;

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

static void createClothGrid(float radius)
{
	const double dist = 0.07;
	const int length = 12; // Should be an even number
	const Vec2f center(0.5, 0.0);
	const Vec2f offset(dist, 0.0);
	const Vec2f offseth(0.0, dist);
	double mass = 0.5;
	radius *= 0.9f;
	//Initialise grid
	for (int i = -(length / 2); i < (length / 2); i++)
	{
		for (int j = -(length / 2); j < (length / 2); j++)
		{
			//add a higher mass to outer particles
			/*if (i == -(length / 2) || i == (length / 2) - 1 || j == -(length / 2) || j == (length / 2) - 1)
				mass = 2.0;
			else
				mass = 0.5;*/

			//set 2 fixed points for the cloth
			if (/*(j == -(length / 2) || j == (length / 2 - 1) || j == 1) &&*/ i == (length / 2 - 1))
			{
				pVector.push_back(new Particle(center + ((float)i * offseth) + ((float)j * offset), mass, radius, true, true));
			}
			else
			{
				pVector.push_back(new Particle(center + ((float)i * offseth) + ((float)j * offset), mass, radius, false, true));
			}

			//pVector.push_back(new Particle(center + ((float)i * offseth) + ((float)j * offset), mass, true));
			//add remaining points
			//if ((j == -(length / 2) || j == (length / 2 - 1) || j == -1) && i == (length / 2 - 1))
			//{
				//constraints.push_back(new WireConstraint(pVector[pVector.size() - 1], pVector[pVector.size() - 1]->m_ConstructPos, 0));
				//constraints.push_back(new WireConstraint(pVector[pVector.size() - 1], pVector[pVector.size() - 1]->m_ConstructPos, 1));
				//constraints.push_back(new PointConstraint(pVector[pVector.size() - 1], pVector[pVector.size() - 1]->m_ConstructPos));
			//}
		}
	}

	for (int i = 0; i < pVector.size(); i++)
	{
		// Add gravity
		//forces.push_back(new GravityForce(pVector[i]));

		// Add drag
		forces.push_back(new DragForce(pVector[i]));
	}

	//Add spring force
	for (int i = 0; i < pVector.size() - 1; i++)
	{
		//add horizontal springs
		if (!((i + 1) % length == 0))
			forces.push_back(new SpringForce(pVector[i], pVector[i + 1], dist, 20.04, 2.25));
		if (i < pVector.size() - length)
		{
			//add vertical springs
			forces.push_back(new SpringForce(pVector[i], pVector[i + length], dist, 20.04, 2.25));

			//add diagonal springs
			if (!((i + 1) % length == 0))
				forces.push_back(new SpringForce(pVector[i + length], pVector[i + 1], dist * sqrt(2), 20.06, 2.5));
			if (!((i) % length == 0))
				forces.push_back(new SpringForce(pVector[i + length], pVector[i - 1], dist * sqrt(2), 20.06, 2.5));

			//add horizontal damping springs
			if (i < pVector.size() - (2 * length))
			{
				forces.push_back(new SpringForce(pVector[i], pVector[i + 2 * length], 2 * dist, 20.02, 2.1, false));

				//if (pVector[i + 2]->m_isFixed == false)
				//	forces.push_back(new AngularForce(pVector[i], pVector[i + 1], pVector[i + 2], 180, 0.005, 0.05));
			}
		}
		//add vertical damping springs
		if (!((i + 2) % length == 0) && !((i + 1) % length == 0))
		{
			forces.push_back(new SpringForce(pVector[i], pVector[i + 2], 2 * dist, 10.02, 2.1, false));
			//if (pVector[i + 2]->m_isFixed == false)
			//	forces.push_back(new AngularForce(pVector[i], pVector[i + 1], pVector[i + 2], 180, 0.005, 0.05));
		}

	}

	//Add mouse force
	/*if (enableMouse)
	{
		mouseForce = new MouseForce(pVector[0], Vec2f(0, 0), dist, 1.0, 1.0);
		forces.push_back(mouseForce);
	}*/

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

static void init_system(void)
{
	const float dist = 0.04;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);

	const float radius = dist * 1.15f;
	
	if (enableCloth)
	{
		createClothGrid(radius);
	}

	// Initialise fluid
	for (int i = 0; i < 1 / dist; i++)
	{
		for (int j = 0; j < 1.3 / dist; j++)
		{
			//if(j == 5)
			//	pVector.push_back(new Particle(Vec2f(-0.95 + ((j % 2) * 0.02) + dist * i, 0.25 - dist * j), 0.4f, radius, true));
			//else
			pVector.push_back(new Particle(Vec2f(-0.92 + ((j % 2) * 0.02) + dist * i, 0.4 - dist * j), 0.4f, radius));
			//	forces.push_back(new GravityForce(pVector[pVector.size() - 1]));
			//	forces.push_back(new DragForce(pVector[pVector.size() - 1]));
			//forces.push_back(new WallForce(pVector[pVector.size() - 1]));
		}
	}

	fluidContainer = new FluidContainer(radius, 2.0f, 2.0f);

	// Create boundary particles
	const float density = 0.03;
	for (int i = 0; i < 1.96 / density; i++)
	{
		for (int j = 0; j < 1.96 / density; j++)
		{
			if (j < 2 || j > int(1.9 / density) || i < 2 || i > int(1.9 / density))
			{
				//fluidContainer->m_BoundaryParticles.push_back(new Particle(Vec2f(-0.98 + density * i, -0.98 + density * j), 0.4f, radius, true, true));
				pVector.push_back(new Particle(Vec2f(-0.98 + density * i, -0.98 + density * j), 0.4f, radius, true, true));
			}
		}
	}


	//rigidBodies.push_back(new Box(Vec2f(-0.4, 0.5), 3, 0.45, 0.15));

	if (enableMoreBodies)
	{
		rigidBodies.push_back(new Box(Vec2f(-0.2, 0.7), 3, 0.2, 0.2));
		//forces.push_back(new GravityForce(rigidBodies[rigidBodies.size() - 1]));

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
	
	if (!enableMoreBodies)
	{
		rigidBodies.push_back(new Box(center, 3, 0.5, 0.5));
		rigidBodies.push_back(new Box(Vec2f(0.35, -0.4), 3, 0.25, 0.25));
		forces.push_back(new DragForce(rigidBodies[0]));
		forces.push_back(new DragForce(rigidBodies[1]));
		forces.push_back(new GravityForce(rigidBodies[0]));
		forces.push_back(new GravityForce(rigidBodies[1]));
	}

	fluidContainer = new FluidContainer(dist / 2, 2.0f, 2.0f);


	if (enableMoreBodies)
	{
		rigidBodies.push_back(new Box(Vec2f(0.40, 0.7), 4, 0.25, 0.25));
		rigidBodies.push_back(new Box(Vec2f(0.35, 0.4), 4, 0.25, 0.25));
		rigidBodies.push_back(new Box(Vec2f(0, -0.60), 4, 0.60, 0.60));
		rigidBodies.push_back(new Box(Vec2f(0, -0.95), 4, 0.1, 0.1));
		forces.push_back(new GravityForce(rigidBodies[0]));
		forces.push_back(new GravityForce(rigidBodies[1]));
		forces.push_back(new GravityForce(rigidBodies[2]));
		forces.push_back(new GravityForce(rigidBodies[3]));
	}

	//Boundaries (Static rigid bodies)
	rigidBodies.push_back(new Box(Vec2f(0.0, -1.5), 1, 2, 1, true));
	rigidBodies.push_back(new Box(Vec2f(0.0, 1.5), 1, 2, 1, true));
	rigidBodies.push_back(new Box(Vec2f(1.5, 0.0), 1, 1, 2, true));
	rigidBodies.push_back(new Box(Vec2f(-1.5, 0.0), 1, 1, 2, true));

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
		//pVector[ii]->drawSurface();
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

		break;
	case '4':
		enableRenderFluid = !enableRenderFluid;
		break;
	case '5':
		enableStandard = true;
		enableBodies = false;
		enableMoreBodies = false;
		enableCloth = false;
		
		free_data();
		load_Scene();
		break;
	case '6':
		enableStandard = true;
		enableBodies = false;
		enableMoreBodies = false;
		enableCloth = true;
		
		free_data();
		load_Scene();
		break;
	case '7':
		enableStandard = true;
		enableBodies = false;
		enableMoreBodies = true;
		enableCloth = false;
		
		free_data();
		load_Scene();
		break;
	case '8':
		enableBodies = true;
		enableStandard = false;
		enableMoreBodies = false;
		enableCloth = false;
		
		free_data();
		load_Scene();
		break;
	case '9':
		enableBodies = true;
		enableStandard = false;
		enableMoreBodies = true;
		enableCloth = false;
		
		free_data();
		load_Scene();
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

	if (enableRenderFluid)
	{
		fluidContainer->RenderFluid(pVector, 8.0f);
	}
	else if(enableDrawParticles)
	{
		draw_particles();
	}
	//fluidContainer->draw();
	
	draw_rigidBodies();

	draw_forces();

	post_display();
}


/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window(void)
{
	//glutInitContextVersion(3, 0);
	//glutInitContextFlags(GLUT_FORWARD_COMPATIBLE | DEBUG);
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

//	glEnable(GL_POLYGON_SMOOTH);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glEnable(GL_BLEND);
	glEnable(GL_POINT_SMOOTH);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

	pre_display();

	glutKeyboardFunc(key_func);
	glutMouseFunc(mouse_func);
	glutMotionFunc(motion_func);
	glutReshapeFunc(reshape_func);
	glutIdleFunc(idle_func);
	glutDisplayFunc(display_func);

	// Initialize glew
	if (glewInit())
		cout << "Failed to initialize glew" << endl;

	fluidContainer->SetupFluidRendering();
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
	printf("\t 4 : Toggle Fluid / Particle rendering \n");
	printf("\t 5 : Standard Fluid \n");
	printf("\t 6 : Fluid with Cloth \n");
	printf("\t 7 : Fluid with Rigid Body \n");
	printf("\t 8 : Simple Rigid Body \n");
	printf("\t 9 : Multiple Rigid Bodies \n");
	printf("\t Dump frames by pressing the 'd' key\n");
	printf("\t Quit by pressing the 'q' key\n");

	load_Scene();

	win_x = 512;
	win_y = 512;
	open_glut_window();

	glutMainLoop();

	exit(0);
}