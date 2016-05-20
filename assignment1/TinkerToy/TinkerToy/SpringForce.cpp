#include "SpringForce.h"
//#include <GL/glut.h>
#include "GLUtil.h"

SpringForce::SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd) : 
	m_p1(p1), m_p2(p2), m_dist(dist), m_ks(ks), m_kd(kd) {}

void SpringForce::draw() 
{
	glBegin(GL_LINES);
	glColor3f(0.6, 0.7, 0.8);
	glVertex2f(m_p1->m_Position[0], m_p1->m_Position[1]);
	glColor3f(0.6, 0.7, 0.8);
	glVertex2f(m_p2->m_Position[0], m_p2->m_Position[1]);
	glEnd();

	// calculate positional and velocity differences
	Vec2f positionDiff = m_p1->m_Position - m_p2->m_Position;
	Vec2f velocityDiff = m_p1->m_Velocity - m_p2->m_Velocity;
	velocityDiff = velocityDiff * 3.0f;
	//velocityDiff = -(velocityDiff / norm(velocityDiff)) * 0.3f;


	glBegin(GL_LINES);
	glColor3f(0.1, 0.7, 0.8);
	glVertex2f(m_p1->m_Position[0], m_p1->m_Position[1]);
	glColor3f(0.1, 0.7, 0.8);
	glVertex2f(m_p1->m_Position[0] + velocityDiff [0], m_p1->m_Position[1] + velocityDiff[1]);
	glEnd();
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

	double distanceDiff =  distance - m_dist;
	double stiffness = m_ks * distanceDiff;
	
	double dotdiv = dotProduct / distance;
	
	// calculate result force
	//Vec2f result = (stiffness + m_kd * dotdiv);
	float scalar = (m_ks * (distance - m_dist) + m_kd * (dotProduct / distance));
	//Vec2f result = (m_ks * (distanceDiff) + 0 * (dotProduct / distance));
	Vec2f result = scalar * (positionDiff / distance);

	// apply force to both particles
	m_p1->m_Force -= result;
	m_p2->m_Force += result;
}