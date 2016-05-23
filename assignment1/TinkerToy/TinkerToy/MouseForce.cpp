#include "MouseForce.h"

#include "GLUtil.h"

MouseForce::MouseForce(Particle * p, Vec2f mousePos, double dist, double ks, double kd) :
	m_p(p), m_dist(dist), m_mousePos(mousePos), m_ks(ks), m_kd(kd) {
	selected = false;
}


void MouseForce::draw()
{
	Vec2f direction = m_p->m_Position;
	direction = m_mousePos;
	direction = (direction / norm(direction)) * 0.2f;
	glBegin(GL_LINES);
	glColor3f(0.2, 0.4, 0.4);
	glVertex2f(m_p->m_Position[0], m_p->m_Position[1]);
	glColor3f(0.2, 0.4, 0.4);
	glVertex2f(m_p->m_Position[0] + direction[0], m_p->m_Position[1] + direction[1]);
	glEnd();
}

void MouseForce::apply()
{
	if (selected)
	{
		// calculate positional and velocity differences
 		Vec2f positionDiff = m_p->m_Position - m_mousePos;			//l
		Vec2f velocityDiff = m_p->m_Velocity;			//i

																			// calculate distance
		float distance = sqrt(positionDiff[0] * positionDiff[0] + positionDiff[1] * positionDiff[1]);

		// calculate dotproduct
		float dotProduct = positionDiff[0] * velocityDiff[0] + positionDiff[1] * velocityDiff[1]; //velocityDiff * positionDiff;

																									// calculate result force
																									//Vec2f result = (stiffness + m_kd * dotdiv);
		float scalar = (m_ks * (distance - m_dist) + m_kd * (dotProduct / distance));
		Vec2f result = scalar * (positionDiff / distance);

		// apply force to both particles
		m_p->m_Force -= result;
	}
}

void MouseForce::newMousePosition(Vec2f mousePos)
{
	m_mousePos = mousePos;
}

void MouseForce::selectParticle(Particle * p)
{
	m_p = p;
	selected = true;
}
void MouseForce::clearParticle()
{
	selected = false;
}
