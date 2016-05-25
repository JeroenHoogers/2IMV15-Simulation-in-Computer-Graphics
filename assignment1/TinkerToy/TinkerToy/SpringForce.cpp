#include "SpringForce.h"
//#include <GL/glut.h>
#include "GLUtil.h"

SpringForce::SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd, bool draw) : 
	m_p1(p1), m_p2(p2), m_dist(dist), m_ks(ks), m_kd(kd), m_drawEnabled(draw) {}

void SpringForce::draw() 
{
	if (m_drawEnabled)
	{
		Vec2f positionDiff = m_p1->m_Position - m_p2->m_Position;
		float distance = sqrt(positionDiff[0] * positionDiff[0] + positionDiff[1] * positionDiff[1]);
		float stretch = (distance / m_dist) - 1;

		// Color changes as the spring distance changes
		// blue = rest distance
		// red = > rest distance

		glBegin(GL_LINES);
		glColor3f(0.5 + stretch, 0.3, 1 - stretch * 2);
		glVertex2f(m_p1->m_Position[0], m_p1->m_Position[1]);
		glColor3f(0.5 + stretch, 0.3, 1 - stretch * 2);
		glVertex2f(m_p2->m_Position[0], m_p2->m_Position[1]);
		glEnd();
	}
}

void SpringForce::apply()
{
	// calculate positional and velocity differences
	Vec2f positionDiff = m_p1->m_Position - m_p2->m_Position;			//l
	Vec2f velocityDiff = m_p1->m_Velocity - m_p2->m_Velocity;			//i
	
	// calculate distance
	float distance = sqrt(positionDiff[0] * positionDiff[0] + positionDiff[1] * positionDiff[1]);
	
	// calculate dotproduct
	float dotProduct = positionDiff[0] * velocityDiff[0] + positionDiff[1] * velocityDiff[1]; //velocityDiff * positionDiff;
	
	// calculate result force
	//Vec2f result = (stiffness + m_kd * dotdiv);
	float scalar = (m_ks * (distance - m_dist) + m_kd * (dotProduct / distance));
	Vec2f result = scalar * (positionDiff / distance);

	// apply force to both particles
	m_p1->m_Force -= result;
	m_p2->m_Force += result;
}