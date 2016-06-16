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
	float bounceFactor = 0.65f;

	bool collision = false;
	Vec2f n = Vec2f(0, 0);
	//right wall
	if (m_p->m_Position[0] > 1.0f) {
		n = Vec2f(-1, 0);
		//reverse the velocity
		m_p->m_Position[0] = 1.0f;

		collision = true;
	}

	//left wall
	if (m_p->m_Position[0] < -1.0f) {
		n = Vec2f(1, 0);
		m_p->m_Position[0] = -1.0f;
		collision = true;
	}

	//lower wall
	if (m_p->m_Position[1] < -1.0f) 
	{
		n = Vec2f(0, 1);

		// clamp position
		m_p->m_Position[1] = -1.0f;
		collision = true;
	}

	//upper wall
	if (m_p->m_Position[1] > 1.0f) 
	{
		n = Vec2f(0, -1);
		m_p->m_Position[1] = 1.0f;

		collision = true;
	}

	if (collision)
	{
		Vec2f Vn = (n * m_p->m_Velocity) * n;
		Vec2f Vt = m_p->m_Velocity - Vn;
		m_p->m_Velocity = Vt - bounceFactor * Vn;
	}

}
