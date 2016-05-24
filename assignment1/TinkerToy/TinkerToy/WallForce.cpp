#include "WallForce.h"

WallForce::WallForce(Particle* p) :
	m_p(p)
{
}

void WallForce::draw()
{

}

void WallForce::apply()
{
	//right wall
	if (m_p->m_Position[0] > 1.0f) {
		//reverse the velocity
		m_p->m_Velocity[0] = -abs(m_p->m_Velocity[0]) * 0.8f;
		m_p->m_Force[0] = -abs(m_p->m_Force[0]) * 0.8f;
	}

	//left wall
	if (m_p->m_Position[0] < -1.0f) {
		//reverse the velocity
		m_p->m_Velocity[0] = abs(m_p->m_Velocity[0]) * 0.8f;
		m_p->m_Force[0] = abs(m_p->m_Force[0]) * 0.8f;
	}

	//upper wall
	if (m_p->m_Position[1] < -1.0f) {
		//reverse the velocity
		m_p->m_Velocity[1] = abs(m_p->m_Velocity[1]) * 0.8f;
		m_p->m_Force[1] = abs(m_p->m_Force[1]) * 0.8f;
	}

	//lower wall
	if (m_p->m_Position[1] > 1.0f) {
		//reverse the velocity
		m_p->m_Velocity[1] = -abs(m_p->m_Velocity[1]) * 0.8f;
		m_p->m_Force[1] = -abs(m_p->m_Force[1]) * 0.8f;
	}
}
