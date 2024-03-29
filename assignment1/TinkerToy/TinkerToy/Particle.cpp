#include "Particle.h"
//#include <GL/glut.h>
#include "GLUtil.h"

Particle::Particle(const Vec2f & ConstructPos, float mass, bool isFixed) :
	m_ConstructPos(ConstructPos), m_Position(Vec2f(0.0, 0.0)), m_Velocity(Vec2f(0.0, 0.0)), m_index(-1), m_Mass(mass), m_isFixed(isFixed)
{
}

Particle::~Particle(void)
{
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
