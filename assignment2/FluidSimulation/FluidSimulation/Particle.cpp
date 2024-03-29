#include "Particle.h"
//#include <GL/glut.h>
#include "GLUtil.h"

Particle::Particle(const Vec2f & ConstructPos, float mass, float radius, bool isFixed, bool isBoundary) :
	m_ConstructPos(ConstructPos), m_Position(Vec2f(0.0, 0.0)), m_Velocity(Vec2f(0.0, 0.0)), m_index(-1), m_Mass(mass), m_Radius(radius), m_isFixed(isFixed), m_GridId(-1), m_isBoundary(isBoundary)
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

void Particle::drawSurface()
{
	glBegin(GL_TRIANGLE_FAN);

	glColor4f(1.0, 1.0, 1.0, 1.0);
	////glColor4f(0.5, 0.5, 1.0, 0.5);
	glVertex2f(m_Position[0], m_Position[1]);
	glColor4f(1.0, 1.0, 1.0, 0.1);


	float size = 0.05f;

	//glColor4f(0.4, 0.3, 1.0, 0.0);
	for (int i = 0; i < 360; i = i + 18)
	{
		float degInRad = i * M_PI / 180;
		glVertex2f(m_Position[0] + cos(degInRad) * size, m_Position[1] + sin(degInRad) *size);
	}

	glVertex2f(m_Position[0] + cos(0) * size, m_Position[1] + sin(0) * size);
	glEnd();
}

void Particle::draw(bool renderFluid)
{
	const double h = 0.02;
	
	//glColor3f(m_Pressure, 1 - m_Pressure, 1 - m_Pressure);
	//glBegin(GL_QUADS);
	//glVertex2f(m_Position[0]-h/2.0, m_Position[1]-h/2.0);
	//glVertex2f(m_Position[0]+h/2.0, m_Position[1]-h/2.0);
	//glVertex2f(m_Position[0]+h/2.0, m_Position[1]+h/2.0);
	//glVertex2f(m_Position[0]-h/2.0, m_Position[1]+h/2.0);
	//glEnd();

	// Draw boundary particles
	if (m_isBoundary)
	{
		return;
		glColor4f(0.3, 1.0, 0.3, 1.0);
		if(m_isActive)
			glColor4f(1.0, 0.3, 0.3, 1.0);
			
		glPointSize(2.0f);

		glBegin(GL_POINTS);
		glVertex2f(m_Position[0], m_Position[1]);
		glEnd();
	}
	else // Draw fluid particles
	{
		if (!renderFluid)
		{
			glBegin(GL_LINE_LOOP);
			glColor4f(1.0, 1.0, 1.0, 0.05);
			for (int i = 0; i < 360; i = i + 18)
			{
				float degInRad = i * M_PI / 180;
				glVertex2f(m_Position[0] + cos(degInRad)*m_Radius, m_Position[1] + sin(degInRad)*m_Radius);
			}
			glEnd();

			glColor3f((m_Pressure * 0.8) + 0.4f, 0.3f, 1.0f - (m_Pressure * 0.8));

			glPointSize(8.0f);

			glBegin(GL_POINTS);
			glVertex2f(m_Position[0], m_Position[1]);
			glEnd();
		}
		else
		{
			//glBegin(GL_TRIANGLE_FAN);

			float v = sqrt(m_Velocity * m_Velocity) * 0.3;
			float p = (0.8 - m_Pressure) * 0.2;
			glColor4f(0.3, 0.3, 1.0, 1.0);
			////glColor4f(0.5, 0.5, 1.0, 0.5);
			//glVertex2f(m_Position[0], m_Position[1]);

			////glColor4f((m_Pressure)+0.4f, 0.4f, 1.0f - (m_Pressure), 0.4);


			//float size = 0.035f;

			////glColor4f(0.4, 0.3, 1.0, 0.0);
			//for (int i = 0; i < 360; i = i + 50)
			//{
			//	float degInRad = i * M_PI / 180;
			//	glVertex2f(m_Position[0] + cos(degInRad) * size, m_Position[1] + sin(degInRad) *size);
			//}

			//glVertex2f(m_Position[0] + cos(0) * size, m_Position[1] + sin(0) * size);
			//glEnd();
			glPointSize(12.0f + 0.1 * (m_Density - 260));
			glBegin(GL_POINTS);
			glVertex2f(m_Position[0], m_Position[1]);
			glEnd();
		}
	}
}
float Particle::getMass()
{
	return m_Mass;
}

Vec2f Particle::getVelocity()
{
	return m_Velocity;
}
void Particle::setVelocity(Vec2f v)
{
	m_Velocity = v;
}

Vec2f Particle::getPosition()
{
	return m_Position;
}

void Particle::setPosition(Vec2f p)
{
	m_Position = p;
}

void Particle::addForce(Vec2f force)
{
	m_Force += force;
}
