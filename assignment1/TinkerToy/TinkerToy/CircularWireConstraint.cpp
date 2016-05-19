#include "CircularWireConstraint.h"
//#include <GL/glut.h>
#include "GLUtil.h"

#define PI 3.1415926535897932384626433832795

static void draw_circle(const Vec2f & vect, float radius)
{
	glBegin(GL_LINE_LOOP);
	glColor3f(0.0,1.0,0.0); 
	for (int i=0; i<360; i=i+18)
	{
		float degInRad = i*PI/180;
		glVertex2f(vect[0]+cos(degInRad)*radius,vect[1]+sin(degInRad)*radius);
	}
	glEnd();
}

CircularWireConstraint::CircularWireConstraint(Particle *p, const Vec2f & center, const double radius) :
	m_p(p), m_center(center), m_radius(radius) {}

void CircularWireConstraint::draw()
{
	draw_circle(m_center, m_radius);
}

float CircularWireConstraint::getC()
{
	//C(x, y, x_c, y_c) = Pow(x - x_c) + Pow(y - y_c) - Pow(r)

	//Calculate difference between particles
	Vec2f difference = m_p->m_Position - m_center;

	//Calculate C(x, y, x_c, y_c)
	float result = ((difference[0] * difference[0]) + (difference[1] * difference[1]) - (m_radius * m_radius));

	//m_p->m_Force -= result;
	return result;
}

float CircularWireConstraint::getCd()
{
	// TODO: implement
	return 0;
}