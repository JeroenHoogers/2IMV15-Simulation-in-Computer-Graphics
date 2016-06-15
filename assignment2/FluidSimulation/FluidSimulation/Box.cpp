#include "Box.h"



Box::Box(const Vec2f constructPos, float density, float height, float width) : RigidBody(constructPos)
{
	m_Width = width;
	m_Height = height;

	Vec2f p = constructPos;

	// Add vertices
	m_Vertices = vector<Particle*>();
	m_Vertices.push_back(new Particle(Vec2f(p[0] - width / 2, p[1] - height / 2)));
	m_Vertices.push_back(new Particle(Vec2f(p[0] + width / 2, p[1] - height / 2)));
	m_Vertices.push_back(new Particle(Vec2f(p[0] + width / 2, p[1] + height / 2)));
	m_Vertices.push_back(new Particle(Vec2f(p[0] - width / 2, p[1] + height / 2)));

	
	m_Mass = density * width * height;

	calculateInertia();

}

void Box::calculateInertia()
{
	m_Inertia = m_Mass * (pow(m_Width, 2) + pow(m_Width, 2));
}

Box::~Box()
{
}
