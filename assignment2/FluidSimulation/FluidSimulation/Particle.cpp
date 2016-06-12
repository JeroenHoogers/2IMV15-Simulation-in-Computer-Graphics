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

float Particle::getW(float r)
{
	// Default kernel function
	// (h^2 - r^2)^3 iff 0 <= r <= h
	// otherwise 0
	if (0 <= r && r <= m_Radius)
		return pow((m_Radius*m_Radius) - (r*r), 3);
	else
		return 0;
}

double Particle::getWPoly6(double r)
{
	// Default kernel function
	// (h^2 - r^2)^3 iff 0 <= r <= h
	// otherwise 0

	double r2 = r * r;
	if (r <= m_Radius * m_Radius)
		return 4.0 / (M_PI * pow(m_Radius, 8)) * pow(m_Radius * m_Radius - r2, 3);
	else
		return 0;
}

Vec2f Particle::getWSpikyGrad(Vec2f r)
{
	// Default kernel function
	// -30 / (PI * h^5) * (h-l)^2 * (r/|r|)
	// otherwise 0
	// Calculate length 
	float l = sqrt(r[0] * r[0] + r[1] * r[1]);
	if(l > 0)
		r = r / l;

	if (0 <= l && l <= m_Radius)
		return -30.0f / ((float)M_PI * pow(m_Radius, 5)) * (m_Radius - l) * (m_Radius - l) * r;
	else
		return Vec2f(0, 0);
}


float Particle::getWGrad(float r)
{
	// Gradient of the kernel function
	// -6(h^2 - r^2)^2 iff 0 <= r <= h
	// otherwise 0
	if (0 <= r && r <= m_Radius)
		return -6 * r * pow((m_Radius*m_Radius) - (r*r), 2);
	else
		return 0;
}

float Particle::getWLaplacian(float r)
{
	// Gradient of the kernel function
	// -12(h^2 - r^2)^2 iff 0 <= r <= h
	// otherwise 0
	if (0 <= r && r <= m_Radius)
		return -12 * (pow(m_Radius, 4)  - 4 * (pow(m_Radius,2) * pow(r,2) + 3 * pow(r,4)));
	else
		return 0;
}


float Particle::getWVisLaplacian(float r)
{
	// Gradient of the kernel function
	// -12(h^2 - r^2)^2 iff 0 <= r <= h
	// otherwise 0
	if (0 <= r && r <= m_Radius)
		return (-(9 * pow(m_Radius, 3) * r) / 2) + (4 / pow(m_Radius,2)) + (m_Radius / (2 * r));
	else
		return 0;
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
	
	glColor3f(m_Pressure, 1 - m_Pressure, 1 - m_Pressure);
	glBegin(GL_QUADS);
	glVertex2f(m_Position[0]-h/2.0, m_Position[1]-h/2.0);
	glVertex2f(m_Position[0]+h/2.0, m_Position[1]-h/2.0);
	glVertex2f(m_Position[0]+h/2.0, m_Position[1]+h/2.0);
	glVertex2f(m_Position[0]-h/2.0, m_Position[1]+h/2.0);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glColor3f(0.0, 1.0, 0.0);
	for (int i = 0; i<360; i = i + 18)
	{
		float degInRad = i*M_PI / 180;
		glVertex2f(m_Position[0] + cos(degInRad)*m_Radius, m_Position[1] + sin(degInRad)*m_Radius);
	}
	glEnd();
}
