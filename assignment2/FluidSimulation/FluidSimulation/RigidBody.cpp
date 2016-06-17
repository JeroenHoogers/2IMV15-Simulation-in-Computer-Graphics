#include "RigidBody.h"
#include "GLUtil.h"
#include "Utility.h"


RigidBody::RigidBody(const Vec2f & ConstructPos, bool isFixed) :
	m_ConstructPos(ConstructPos), m_isFixed(isFixed)
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
	m_AngularVelocity = 0;
	m_Orientation = 0;
	updateRotation(0);

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
		glVertex2f(m_Vertices[i]->m_Position[0] + m_Position[0], m_Vertices[i]->m_Position[1] + m_Position[1]);
	}
	glEnd();

	// Draw vertices
	//for (int i = 0; i < m_Vertices.size(); i++)
	//{
	//	m_Vertices[i]->draw();
	//}
}
vector<float> RigidBody::getExtremes()
{
	//Extremes in world coordinates
	vector<float> extremes = vector<float>();
	extremes.push_back(-2.0f);
	extremes.push_back(-2.0f);
	extremes.push_back(2.0f);
	extremes.push_back(2.0f);
	for (int i = 0; i < m_Vertices.size(); i++)
	{
		//matrix currentVertex = matrix(2, 1);
		//currentVertex.setValue(0, 0, /*rigidBodies[i]->m_Position[0]*/ -m_Vertices[i]->m_Position[0]);
		//currentVertex.setValue(1, 0, /*rigidBodies[i]->m_Position[1]*/ -m_Vertices[i]->m_Position[1]);
		//currentVertex.printMatrix();
		//cout << endl;
		//matrix result = *m_Rotation * currentVertex;
		if (m_Vertices[i]->m_Position[0] + m_Position[0] > extremes[0])
			extremes[0] = m_Vertices[i]->m_Position[0] + m_Position[0];
		if (m_Vertices[i]->m_Position[1] + m_Position[1] > extremes[1])
			extremes[1] = m_Vertices[i]->m_Position[1] + m_Position[1];
		if (m_Vertices[i]->m_Position[0] + m_Position[0] < extremes[2])
			extremes[2] = m_Vertices[i]->m_Position[0] + m_Position[0];
		if (m_Vertices[i]->m_Position[1] + m_Position[1] < extremes[3])
			extremes[3] = m_Vertices[i]->m_Position[1] + m_Position[1];
		//currentVertex.release();
		//result.release();
	}
	return extremes;
}

Vec2f RigidBody::BroadPhase(RigidBody* other)
{
	//updateRotation(-m_Orientation);
	vector<float> extremes = getExtremes();

	for (int i = 0; i < other->m_Vertices.size(); i++)
	{
		//Rotate other body to current bodies
		/*matrix currentVertex = matrix(2, 1);
		currentVertex.setValue(0, 0, -other->m_Vertices[i]->m_Position[0] - other->m_Position[0] + m_Position[0]);
		currentVertex.setValue(1, 0, -other->m_Vertices[i]->m_Position[1] - other->m_Position[1] + m_Position[1]);
		matrix result = *m_Rotation * currentVertex;
		float resultx = result.getValue(0, 0);
		float resulty = result.getValue(1, 0);*/
		if (extremes[0] > other->m_Vertices[i]->m_Position[0] + other->m_Position[0]//(result.getValue(0, 0))
			&& extremes[1] > other->m_Vertices[i]->m_Position[1] + other->m_Position[1]//(result.getValue(1, 0))
			&& extremes[2] < other->m_Vertices[i]->m_Position[0] + other->m_Position[0]//(result.getValue(0, 0))
			&& extremes[3] < other->m_Vertices[i]->m_Position[1] + other->m_Position[1])//(result.getValue(1, 0)))
		{
			return other->m_Vertices[i]->m_Position + other->m_Position;
		}
		//currentVertex.release();
		//result.release();
	}
	//updateRotation(m_Orientation);
	return Vec2f(0, 0);
}


void RigidBody::generateGhostParticles()
{
	float density = 0.02;

	// Generate ghost particles on the edges of our object.
	for (int i = 0; i < m_Vertices.size(); i++)
	{
		Vec2f v1;

		if (i == 0)
			v1 = m_Vertices[m_Vertices.size() - 1]->getPosition();
		else
			v1 = m_Vertices[i - 1]->getPosition();

		Vec2f v2 = m_Vertices[i]->getPosition();

		//v1 = v1 * 0.8f + m_Position;
		//v2 = v2 * 0.8f + m_Position;

		Vec2f dir = v2 - v1;

		float dist = sqrt(dir[0] * dir[0] + dir[1] * dir[1]);

		// Normalise direction
		dir /= dist;

		for (int j = 1; j < dist / density; j++)
		{
			// Generate ghost particle
			m_GhostParticles.push_back(
				new Particle(
					v1 * 0.8f + m_Position + dir * (j * density),
					0.4f, 0.05, true));

			m_GhostParticles[m_GhostParticles.size() - 1]->m_LocalPosition = v1 + dir * (j * density);
		}
	}
}

void RigidBody::updateGhostParticles()
{
	// Update ghost particle positions
	for (int i = 0; i < m_GhostParticles.size(); i++)
	{
		// TODO: apply rotation
		// HACK: 0.8 is used to position the particles slightly inside the rigidbody such that the fluid doesn't leave a gap
		m_GhostParticles[i]->m_Position = m_GhostParticles[i]->m_LocalPosition * 0.8f + m_Position;
	}
}


void RigidBody::updateRotation(float angle)
{
	/*m_Rotation->setValue(0, 0, sin(angle));
	m_Rotation->setValue(0, 1, cos(angle));
	m_Rotation->setValue(1, 0, cos(angle));
	m_Rotation->setValue(1, 1, -sin(angle));*/
	m_Rotation->setValue(0, 0, cos(angle));
	m_Rotation->setValue(0, 1, -sin(angle));
	m_Rotation->setValue(1, 0, sin(angle));
	m_Rotation->setValue(1, 1, cos(angle));
}

Vec2f RigidBody::getRadiusVec(Vec2f pos)
{
	return (abs(pos[0] - m_Position[0]), abs(pos[1] - m_Position[1]));
}

float RigidBody::getMass()
{
	return m_Mass;
}

Vec2f RigidBody::getVelocity()
{
	return m_Velocity;
}

void RigidBody::setVelocity(Vec2f v)
{
	m_Velocity = v;
}

Vec2f RigidBody::getPosition()
{
	return m_Position;
}

void RigidBody::setPosition(Vec2f p)
{
	m_Position = p;
}

void RigidBody::addForce(Vec2f force)
{
	m_Force += force;
}