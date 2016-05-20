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

	//Calculate difference vector between the particle and center of the wire circle
	Vec2f difference = m_p->m_Position - m_center;

	//Calculate C(x, y, x_c, y_c)
	float result = ((difference[0] * difference[0]) + (difference[1] * difference[1]) - (m_radius * m_radius));

	return result;
}

float CircularWireConstraint::getCd()
{
	//C(x, y, x_c, y_c) = 2 * (x - x_c) + 2 * (y - y_c)
	Vec2f positionDiff = 2.0f * (m_p->m_Position - m_center);
	Vec2f velocityDiff = (m_p->m_Velocity);

	float result = positionDiff[0] * velocityDiff[0] + positionDiff[1] * velocityDiff[1];

	return result;
}