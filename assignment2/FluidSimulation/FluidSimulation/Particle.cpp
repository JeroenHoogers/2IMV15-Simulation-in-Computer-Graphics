#include "Particle.h"
//#include <GL/glut.h>
#include "GLUtil.h"

Particle::Particle(const Vec2f & ConstructPos, float mass, float radius, bool isFixed) :
	m_ConstructPos(ConstructPos), m_Position(Vec2f(0.0, 0.0)), m_Velocity(Vec2f(0.0, 0.0)), m_index(-1), m_Mass(mass), m_Radius(radius), m_isFixed(isFixed)
{
}

Particle::~Particle(void)
{
}
//
//float Particle::getDensityAt(Vec2f p)
//{
//	// m * W(|r-r_j|,h)
//	float dist = distTo(p);
//	return m_Mass * getW(dist);
//}

float Particle::distTo(Vec2f p)
{
	Vec2f positionDiff = p - m_Position;
	return sqrt(positionDiff[0] * positionDiff[0] + positionDiff[1] * positionDiff[1]);
}

void Particle::reset()
{
	m_Position = m_ConstructPos;
	m_Velocity = Vec2f(0.0, 0.0);
	m_Force = Vec2f(0.0, 0.0);
}

void Particle::draw()
{
	const double h = 0.02;
	
	//glColor3f(m_Pressure, 1 - m_Pressure, 1 - m_Pressure);
	//glBegin(GL_QUADS);
	//glVertex2f(m_Position[0]-h/2.0, m_Position[1]-h/2.0);
	//glVertex2f(m_Position[0]+h/2.0, m_Position[1]-h/2.0);
	//glVertex2f(m_Position[0]+h/2.0, m_Position[1]+h/2.0);
	//glVertex2f(m_Position[0]-h/2.0, m_Position[1]+h/2.0);
	//glEnd();

	bool debugRender = true;
	if (debugRender)
	{
		glBegin(GL_LINE_LOOP);
		glColor4f(1.0, 1.0, 1.0, 0.1);
		for (int i = 0; i < 360; i = i + 18)
		{
			float degInRad = i * M_PI / 180;
			glVertex2f(m_Position[0] + cos(degInRad)*m_Radius, m_Position[1] + sin(degInRad)*m_Radius);
		}
		glEnd();

		glColor3f((m_Pressure * 0.7)+0.4f, 0.3f, 1.0f - (m_Pressure * 0.7));
		
		glPointSize(8.0f);

		glBegin(GL_POINTS);
		glVertex2f(m_Position[0], m_Position[1]);
		glEnd();
	}
	else
	{

		glBegin(GL_TRIANGLE_FAN);
		glColor4f(0.45, 0.45, 1.0, 0.4);
		glVertex2f(m_Position[0], m_Position[1]);

		//glColor4f((m_Pressure)+0.4f, 0.4f, 1.0f - (m_Pressure), 0.4);

		float size = 0.04f;
		glColor4f(0.3, 0.3 , 0.9, 0.15);
		//glColor4f(0.4, 0.3, 1.0, 0.0);
		for (int i = 0; i < 360; i = i + 18)
		{
			float degInRad = i * M_PI / 180;
			glVertex2f(m_Position[0] + cos(degInRad) * size, m_Position[1] + sin(degInRad) *size);
		}
		glVertex2f(m_Position[0] + cos(0) * size, m_Position[1] + sin(0) * size);
		glEnd();




	//	glColor4f((m_Pressure)+0.4f, 0.4f, 1.0f - (m_Pressure), 0.3);
		//glPointSize(15.0f);

		//glBegin(GL_POINTS);
		//glVertex2f(m_Position[0], m_Position[1]);
		//glEnd();
	}
}
