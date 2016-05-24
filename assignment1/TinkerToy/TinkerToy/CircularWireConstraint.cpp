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
	m_p(p), m_center(center), m_radius(radius) 
{
}

void CircularWireConstraint::draw()
{
	draw_circle(m_center, m_radius);
}

float CircularWireConstraint::getC()
{
	//C(x, y, x_c, y_c) = Pow(x - x_c) + Pow(y - y_c) - Pow(r)

	//Calculate difference vector between the particle and center of the wire circle
	Vec2f positionDiff = m_p->m_Position - m_center;

	//Calculate C(x, y, x_c, y_c)
	float C = (positionDiff * positionDiff) - (m_radius * m_radius);

	return C;
}

float CircularWireConstraint::getCd()
{
	// Cd(x, y, x_c, y_c) = 2 * (x - x_c) + 2 * (y - y_c)
	Vec2f positionDiff = (m_p->m_Position - m_center) * 2.0f;
	Vec2f velocityDiff = (m_p->m_Velocity) * 2.0f;

	float Cd = positionDiff * velocityDiff;

	return Cd;
}

vector<Vec2f> CircularWireConstraint::getJ()
{
	std::vector<Vec2f> J;

	Vec2f positionDiff = (m_p->m_Position - m_center) * 2.0f;
	J.push_back(positionDiff);

	return J;
}

vector<Vec2f> CircularWireConstraint::getJd()
{
	std::vector<Vec2f> Jd;

	Vec2f velocityDiff = m_p->m_Velocity * 2.0f;
	Jd.push_back(velocityDiff);

	return Jd;
}

vector<Particle*> CircularWireConstraint::getParticles()
{
	vector<Particle*> particles;

	particles.push_back(m_p);

	return particles;
}