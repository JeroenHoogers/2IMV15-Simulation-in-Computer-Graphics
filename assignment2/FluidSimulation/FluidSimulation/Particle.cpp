#include "Particle.h"
//#include <GL/glut.h>
#include "GLUtil.h"

Particle::Particle(const Vec2f & ConstructPos, float mass, float radius, bool isFixed) :
	m_ConstructPos(ConstructPos), m_Position(Vec2f(0.0, 0.0)), m_Velocity(Vec2f(0.0, 0.0)), m_index(-1), m_Mass(mass), m_Radius(radius), m_isFixed(isFixed)
{
}

Particle::~Particle(void)
{
}

float Particle::getDensityAt(Vec2f p)
{
	// m * W(|r-r_j|,h)
	float dist = distTo(p);
	return m_Mass * getW(dist);
}

float Particle::distTo(Vec2f p)
{
	Vec2f positionDiff = p - m_Position;
	float distance = sqrt(positionDiff[0] * positionDiff[0] + positionDiff[1] * positionDiff[1]);
	return abs(distance);
}

float Particle::getW(float r)
{
	// Default kernel function
	// (h^2 - r^2)^3 iff 0 <= r <= h
	// otherwise 0
	if (0 <= r && r <= m_Radius)
		return pow((m_Radius*m_Radius) - (r*r), 3);
	else
		return 0;
}


void Particle::reset()
{
	m_Position = m_ConstructPos;
	m_Velocity = Vec2f(0.0, 0.0);
	m_Force = Vec2f(0.0, 0.0);
}

void Particle::draw()
{
	const double h = 0.03;
	glColor3f(1.f, 1.f, 1.f); 
	glBegin(GL_QUADS);
	glVertex2f(m_Position[0]-h/2.0, m_Position[1]-h/2.0);
	glVertex2f(m_Position[0]+h/2.0, m_Position[1]-h/2.0);
	glVertex2f(m_Position[0]+h/2.0, m_Position[1]+h/2.0);
	glVertex2f(m_Position[0]-h/2.0, m_Position[1]+h/2.0);
	glEnd();
}
