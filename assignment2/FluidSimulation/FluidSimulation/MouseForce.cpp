#include "MouseForce.h"

#include "GLUtil.h"

MouseForce::MouseForce(vector<Particle*> particles, Vec2f mousePos, double dist, double ks, double kd) :
	m_particles(particles), m_dist(dist), m_mousePos(mousePos), m_ks(ks), m_kd(kd)
{
	leftMouseDown = false;
	rightMouseDown = false;
}


void MouseForce::draw()
{
	if (leftMouseDown)
	{
		for (int i = 0; i < m_particles.size(); i++)
		{
			if (m_particles[i]->m_isFixed)
				continue;

			glBegin(GL_LINES);
			glColor4f(1, 1, 1, 0.3);
			glVertex2f(m_particles[i]->m_Position[0], m_particles[i]->m_Position[1]);
			glVertex2f(m_mousePos[0], m_mousePos[1]);
			glEnd();
		}
	}
	else if (rightMouseDown)
	{
		for (int i = 0; i < m_rigidbodies.size(); i++)
		{
			glBegin(GL_LINES);
			glColor4f(1, 1, 1, 0.3);
			glVertex2f(m_rigidbodies[i]->m_Position[0], m_rigidbodies[i]->m_Position[1]);
			glVertex2f(m_mousePos[0], m_mousePos[1]);
			glEnd();
		}
	}
}

void MouseForce::apply()
{
	if (leftMouseDown)
	{
		for (int i = 0; i < m_particles.size(); i++)
		{
			// calculate positional and velocity differences
			Vec2f positionDiff = m_particles[i]->m_Position - m_mousePos;			//l
			Vec2f velocityDiff = m_particles[i]->m_Velocity;						//i
																					// calculate distance
			float distance = sqrt(positionDiff[0] * positionDiff[0] + positionDiff[1] * positionDiff[1]);

			// calculate dotproduct
			float dotProduct = positionDiff * velocityDiff;

			float scalar = (m_ks * (distance - m_dist) + m_kd * (dotProduct / distance));

			Vec2f result = scalar * (positionDiff / distance);

			m_particles[i]->m_Force -= result;
		}

																									// calculate result force
																							//Vec2f result = (stiffness + m_kd * dotdiv);
		//// apply force to both particles
		//m_p->m_Force -= result;
	}
	else if (rightMouseDown)
	{
		for (int i = 0; i < m_rigidbodies.size(); i++)
		{
			// calculate positional and velocity differences
			Vec2f positionDiff = m_rigidbodies[i]->m_Position - m_mousePos;			//l
			Vec2f velocityDiff = m_rigidbodies[i]->m_Velocity;						//i
																					// calculate distance
			float distance = sqrt(positionDiff[0] * positionDiff[0] + positionDiff[1] * positionDiff[1]);

			// calculate dotproduct
			float dotProduct = positionDiff * velocityDiff;

			float scalar = (m_ks * (distance - m_dist) + m_kd * (dotProduct / distance));

			Vec2f result = scalar * (positionDiff / distance);

			m_rigidbodies[i]->m_Force -= result;
		}
	}
}

void MouseForce::newMousePosition(Vec2f mousePos)
{
	m_mousePos = mousePos;
}

void MouseForce::selectParticles(vector<Particle*> particles)
{
	m_particles = particles;
	leftMouseDown = true;
}

void MouseForce::clearParticles()
{
	leftMouseDown = false;
}

void MouseForce::selectRigidbodies(vector<RigidBody*> rigidbodies)
{
	m_rigidbodies = rigidbodies;
	rightMouseDown = true;
}

void MouseForce::clearRigidbodies()
{
	rightMouseDown = false;
}
