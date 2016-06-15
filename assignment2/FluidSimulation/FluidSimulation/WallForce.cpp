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
	float bounceFactor = 0.4f;
	//right wall
	if (m_p->m_Position[0] > 1.0f) {
		//reverse the velocity
		m_p->m_Position[0] = 1.0f;
		m_p->m_Velocity[0] *= -bounceFactor;
		//m_p->m_Velocity[0] = -abs(m_p->m_Velocity[0]) * bounceFactor;
		//m_p->m_Force[0] = -abs(m_p->m_Force[0]) * bounceFactor;
	}

	//left wall
	if (m_p->m_Position[0] < -1.0f) {
		//reverse the velocity
		m_p->m_Position[0] = -1.0f;
		m_p->m_Velocity[0] *= -bounceFactor;
		//m_p->m_Velocity[0] = abs(m_p->m_Velocity[0]) * bounceFactor;
		//m_p->m_Force[0] = abs(m_p->m_Force[0]) * bounceFactor;
	}

	//upper wall
	if (m_p->m_Position[1] < -1.0f) {
		//reverse the velocity
		m_p->m_Position[1] = -1.0f;
		m_p->m_Velocity[1] *= -bounceFactor;
		//m_p->m_Velocity[1] = abs(m_p->m_Velocity[1]) * bounceFactor;
		//m_p->m_Force[1] = abs(m_p->m_Force[1]) * bounceFactor;
	}

	//lower wall
	if (m_p->m_Position[1] > 1.0f) {
		//reverse the velocity
		m_p->m_Position[1] = 1.0f;
		m_p->m_Velocity[1] *= -bounceFactor;
		//m_p->m_Velocity[1] = -abs(m_p->m_Velocity[1]) * bounceFactor;
		//m_p->m_Force[1] = -abs(m_p->m_Force[1]) * bounceFactor;
	}
}
