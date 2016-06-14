#include "RigidBody.h"
#include "GLUtil.h"


RigidBody::RigidBody(const Vec2f & ConstructPos) :
	m_ConstructPos(ConstructPos)
{
	m_Velocity = Vec2f(0, 0);
	m_Force = Vec2f(0, 0);
	m_Rotation = new matrix(2, 2);
}


RigidBody::~RigidBody()
{
}

void RigidBody::reset()
{
	m_Position = m_ConstructPos;
	m_Velocity = Vec2f(0.0, 0.0);
	m_Force = Vec2f(0.0, 0.0);

	for (int i = 0; i < m_Vertices.size(); i++)
	{
		m_Vertices[i]->reset();
	}
}

void RigidBody::draw()
{
	// Draw polygon
	glColor3f(0.2, 0.4, 0.6);

	glBegin(GL_POLYGON);
	for (int i = 0; i < m_Vertices.size(); i++)
	{
		glVertex2f(m_Vertices[i]->m_Position[0], m_Vertices[i]->m_Position[1]);
	}
	glEnd();

	// Draw vertices
	for (int i = 0; i < m_Vertices.size(); i++)
	{
		m_Vertices[i]->draw();
	}
}


void RigidBody::updateRotation(float angle)
{
	m_Rotation->setValue(0, 0, cos(angle));
	m_Rotation->setValue(0, 1, -sin(angle));
	m_Rotation->setValue(1, 0, sin(angle));
	m_Rotation->setValue(1, 1, cos(angle));
}