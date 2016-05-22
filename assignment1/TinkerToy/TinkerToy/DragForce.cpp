#include "DragForce.h"

DragForce::DragForce(Particle* p) :
	m_p(p), m_Drag(0.03)
{
}

void DragForce::draw()
{
	
}

void DragForce::apply()
{
	// Apply drag force
	m_p->m_Force -= m_Drag * m_p->m_Velocity;
}
