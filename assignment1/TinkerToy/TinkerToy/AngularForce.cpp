#include "AngularForce.h"
//#include <GL/glut.h>
#include "GLUtil.h"

AngularForce::AngularForce(Particle *p1, Particle * p2, Particle * p3, double angle, double ks, double kd) :
	m_p1(p1), m_p2(p2), m_p3(p3), m_angle(angle), m_ks(ks), m_kd(kd) {}

void AngularForce::draw()
{
	Vec2f positionDiff = m_p1->m_Position - m_p2->m_Position;
	float distance = sqrt(positionDiff[0] * positionDiff[0] + positionDiff[1] * positionDiff[1]);
	float stretch = (distance / m_angle) - 1;

	// Color changes as the spring distance changes
	// blue = rest distance
	// red = > rest distance

	glBegin(GL_LINES);
	glColor3f(0.5 + stretch, 0.3, 1 - stretch * 2);
	glVertex2f(m_p1->m_Position[0], m_p1->m_Position[1]);
	glColor3f(0.5 + stretch, 0.3, 1 - stretch * 2);
	glVertex2f(m_p2->m_Position[0], m_p2->m_Position[1]);

	positionDiff = m_p2->m_Position - m_p3->m_Position;
	distance = sqrt(positionDiff[0] * positionDiff[0] + positionDiff[1] * positionDiff[1]);
	stretch = (distance / m_angle) - 1;

	glColor3f(0.5 + stretch, 0.3, 1 - stretch * 2);
	glVertex2f(m_p2->m_Position[0], m_p2->m_Position[1]);
	glColor3f(0.5 + stretch, 0.3, 1 - stretch * 2);
	glVertex2f(m_p3->m_Position[0], m_p3->m_Position[1]);
	glEnd();

}

void AngularForce::apply()
{
	float pi = 3.1415926f;

	// calculate positional and velocity differences
	Vec2f positionDiff1to2 = m_p1->m_Position - m_p2->m_Position;
	Vec2f positionDiff3to2 = m_p3->m_Position - m_p2->m_Position;
	Vec2f velocityDiff1to2 = m_p1->m_Velocity - m_p2->m_Velocity;
	Vec2f velocityDiff3to2 = m_p3->m_Velocity - m_p2->m_Velocity;

	float distance1to2 = sqrt(positionDiff1to2[0] * positionDiff1to2[0] + positionDiff1to2[1] * positionDiff1to2[1]);
	float distance3to2 = sqrt(positionDiff3to2[0] * positionDiff3to2[0] + positionDiff3to2[1] * positionDiff3to2[1]);
	float distance1to2Sqrt = positionDiff1to2[0] * positionDiff1to2[0] + positionDiff1to2[1] * positionDiff1to2[1];
	float distance3to2Sqrt = positionDiff3to2[0] * positionDiff3to2[0] + positionDiff3to2[1] * positionDiff3to2[1];

	float distanceDot = (positionDiff1to2[0] * positionDiff3to2[0] + positionDiff1to2[1] * positionDiff3to2[1]);

	float distanceTotal = (positionDiff3to2 / distance1to2) * (positionDiff1to2 / distance3to2);

	float cosineSqrt = distanceDot * distanceDot / distance1to2Sqrt / distance3to2Sqrt;
	float cosine = 2 * cosineSqrt - 1;

	/*float arccosDist;
	if (distanceTotal > -1.0 && distanceTotal < 1.0)
		arccosDist = acos(distanceTotal);
	else if (distanceTotal >= 1.0)
		arccosDist = 0;
	else if (distanceTotal <= -1.0)
		arccosDist = pi;*/

	/*float angle;
	float D = (positionDiff1to2[0] * positionDiff3to2[1] - positionDiff3to2[0] * positionDiff1to2[1]);
	if (D < 0)
		angle = m_angle - arccosDist;
	else
		angle = arccosDist - m_angle;*/
	float alpha2 =
		(cosine <= -1) ? pi :
		(cosine >= 1) ? 0 :
		acosf(cosine);

	float result = alpha2 / 2;
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
	float radangle = 0.001f*(angle * pi / 180);

	m_p1->m_Force[0] -= ((m_ks * radangle) / distance1to2 * positionDiff1to2[1] ) + (m_kd * velocityDiff1to2[0]);
	m_p1->m_Force[1] += ((m_ks * radangle) / distance1to2 * positionDiff1to2[0]) - (m_kd * velocityDiff1to2[1]);

	m_p3->m_Force[0] += ((m_ks * radangle) / distance3to2 * positionDiff3to2[1]) - (m_kd * velocityDiff3to2[0]);
	m_p3->m_Force[1] -= ((m_ks * radangle) / distance3to2 * positionDiff3to2[0]) + (m_kd * velocityDiff3to2[1]);
}