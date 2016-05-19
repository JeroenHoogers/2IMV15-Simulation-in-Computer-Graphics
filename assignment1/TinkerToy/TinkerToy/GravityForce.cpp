#include "GravityForce.h"

GravityForce::GravityForce(Particle* particle) :
	m_particle(particle), m_Gravity(Vec2f(0, -0.000000981)) {}

void GravityForce::draw()
{
	Vec2f direction = m_Gravity;
	direction = (direction / norm(direction)) * 0.3f;
	glBegin(GL_LINES);
	glColor3f(0.6, 0.7, 0.4);
	glVertex2f(m_particle->m_Position[0], m_particle->m_Position[1]);
	glColor3f(0.6, 0.7, 0.4);
	glVertex2f(m_particle->m_Position[0] + direction[0], m_particle->m_Position[1] + direction[1]);
	glEnd();

}

void GravityForce::apply()
{
	m_particle->m_Force += m_particle->m_Mass * m_Gravity;
}
