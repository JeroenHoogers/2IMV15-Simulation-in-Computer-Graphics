#include "AngularForce.h"
//#include <GL/glut.h>
#include "GLUtil.h"

AngularForce::AngularForce(Particle *p1, Particle * p2, Particle * p3, double angle, double ks, double kd) :
	m_p1(p1), m_p2(p2), m_p3(p3), m_angle(angle), m_ks(ks), m_kd(kd) {}

void AngularForce::draw()
{

}

void AngularForce::apply()
{
	float pi = 3.1415926f;

	// calculate positional and velocity differences
	Vec2f positionDiff1to2 = m_p1->m_Position - m_p2->m_Position;
	Vec2f positionDiff3to2 = m_p3->m_Position - m_p2->m_Position;
	Vec2f velocityDiff1to2 = m_p1->m_Velocity - m_p2->m_Velocity;
	Vec2f velocityDiff3to2 = m_p3->m_Velocity - m_p2->m_Velocity;

	// calculate distance to ther middle vertex
	float distance1to2 = sqrt(positionDiff1to2[0] * positionDiff1to2[0] + positionDiff1to2[1] * positionDiff1to2[1]);
	float distance3to2 = sqrt(positionDiff3to2[0] * positionDiff3to2[0] + positionDiff3to2[1] * positionDiff3to2[1]);
	float distance1to2Sqrt = positionDiff1to2[0] * positionDiff1to2[0] + positionDiff1to2[1] * positionDiff1to2[1];
	float distance3to2Sqrt = positionDiff3to2[0] * positionDiff3to2[0] + positionDiff3to2[1] * positionDiff3to2[1];


	float distanceDot = (positionDiff1to2[0] * positionDiff3to2[0] + positionDiff1to2[1] * positionDiff3to2[1]);

	float distanceTotal = (positionDiff3to2 / distance1to2) * (positionDiff1to2 / distance3to2);

	float cosineSqrt = distanceDot * distanceDot / distance1to2Sqrt / distance3to2Sqrt;
	float cosine = 2 * cosineSqrt - 1;

	float alphasqr;
	if (cosine <= -1) 
		alphasqr = pi;
	else if (cosine >= 1) 
		alphasqr = 0;
	else 
		alphasqr = acosf(cosine);

	float result = alphasqr / 2;
	result = result * 180. / pi;

	if (distanceDot < 0)
		result = 180 - result;

	float det = (positionDiff1to2[0] * positionDiff3to2[1] - positionDiff1to2[1] * positionDiff3to2[0]);
	if (det < 0)
	{
		result = -result;
	}

	float angle = 0;
	if (result < 0)
	{
		angle = result + m_angle;
	}
	else
	{
		angle = result - m_angle;
	}
	
	//scale the force so it doesn't apply to much force
	angle = 0.001f*angle;

	m_p1->m_Force[0] -= ((m_ks * angle) / distance1to2 * positionDiff1to2[1]) + (m_kd * velocityDiff1to2[0]);
	m_p1->m_Force[1] += ((m_ks * angle) / distance1to2 * positionDiff1to2[0]) - (m_kd * velocityDiff1to2[1]);

	m_p3->m_Force[0] += ((m_ks * angle) / distance3to2 * positionDiff3to2[1]) - (m_kd * velocityDiff3to2[0]);
	m_p3->m_Force[1] -= ((m_ks * angle) / distance3to2 * positionDiff3to2[0]) + (m_kd * velocityDiff3to2[1]);
}