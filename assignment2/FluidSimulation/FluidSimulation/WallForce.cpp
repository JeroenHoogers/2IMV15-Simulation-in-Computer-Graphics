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
	float bounceFactor = 0.7f;

	Vec2f n = Vec2f(0, 0);
	Vec2f Vn = Vec2f(0, 0);
	Vec2f Vt = Vec2f(0, 0);
	bool collision = false;
	
	//right wall
	if (m_p->m_Position[0] > 1.0f) {
		n = Vec2f(-1, 0);
		//reverse the velocity
		m_p->m_Position[0] = 1.0f;

		//m_p->m_Velocity[0] *= -bounceFactor;
		collision = true;

		//m_p->m_Velocity[0] = -abs(m_p->m_Velocity[0]) * bounceFactor;
		//m_p->m_Force[0] = -abs(m_p->m_Force[0]) * bounceFactor;
	}

	//left wall
	if (m_p->m_Position[0] < -1.0f) {
		//reverse the velocity
		n = Vec2f(1, 0);
		m_p->m_Position[0] = -1.0f;
		collision = true;

		//m_p->m_Velocity[0] *= -bounceFactor;
		//m_p->m_Velocity[0] = abs(m_p->m_Velocity[0]) * bounceFactor;
		//m_p->m_Force[0] = abs(m_p->m_Force[0]) * bounceFactor;
	}

	//lower wall
	if (m_p->m_Position[1] < -1.0f) 
	{
		n = Vec2f(0, 1);
		//reverse the velocity
		m_p->m_Position[1] = -1.0f;
		//m_p->m_Velocity[1] *= -bounceFactor;
		collision = true;

	//	m_p->m_Velocity[1] *= -bounceFactor;
		//m_p->m_Velocity[1] = abs(m_p->m_Velocity[1]) * bounceFactor;
		//m_p->m_Force[1] = abs(m_p->m_Force[1]) * bounceFactor;
	}

	//upper wall
	if (m_p->m_Position[1] > 1.0f) 
	{
		n = Vec2f(0, -1);

		//reverse the velocity
		m_p->m_Position[1] = 1.0f;
		//m_p->m_Velocity[1] *= -bounceFactor;
		collision = true;

		//m_p->m_Velocity[1] *= -bounceFactor;
		//m_p->m_Velocity[1] = -abs(m_p->m_Velocity[1]) * bounceFactor;
		//m_p->m_Force[1] = -abs(m_p->m_Force[1]) * bounceFactor;
	}

	if (collision)
	{
		Vn = (n * m_p->m_Velocity) * n;
		Vt = m_p->m_Velocity - Vn;
		m_p->m_Velocity = Vt - bounceFactor * Vn;
	}

}
