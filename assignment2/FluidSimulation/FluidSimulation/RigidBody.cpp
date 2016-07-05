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
	glColor3f(0.9, 0.8, 0.65);

	glBegin(GL_POLYGON);
	for (int i = 0; i < m_Vertices.size(); i++)
	{
		glVertex2f(m_Vertices[i]->m_Position[0] + m_Position[0], m_Vertices[i]->m_Position[1] + m_Position[1]);
	}
	glEnd();
	glColor3f(0.8, 0.4, 0.6);
	glBegin(GL_POINTS);
	//Draw vertices
	for (int i = 0; i < m_ImpactPoints.size(); i++)
	{
		glVertex2f(m_ImpactPoints[i][0], m_ImpactPoints[i][1]);
	}
	glEnd();
	//glColor3f(0.1, 0.8, 0.6);
	//glBegin(GL_LINES);
	////Draw normals
	//for (int i = 0; i < m_Normals.size(); i++)
	//{
	//	Vec2f n = m_NormalPositions[i] + m_Position;
	//	glVertex2f(n[0], n[1]);
	//	n += m_Normals[i] * 0.2f;
	//	glVertex2f(n[0], n[1]);
	//}
	//glEnd();
	// Draw vertices
	//for (int i = 0; i < m_Vertices.size(); i++)
	//{
	//	m_Vertices[i]->draw();
	//}
}


void RigidBody::calculateNormals()
{
	for (int i = 0; i < m_Vertices.size(); i++)
	{
		int first = i;
		int second = (i + 1) % m_Vertices.size();
		Vec2f direction = (m_Vertices[first]->m_ConstructPos - m_Vertices[second]->m_ConstructPos);
		Vec2f normal = Vec2f(-direction[1], direction[0]);
		normal = Util::normalise(normal);
		m_Normals.push_back(normal);
		m_NormalPositions.push_back((m_Vertices[first]->m_ConstructPos + m_Vertices[second]->m_ConstructPos) * 0.5f);
	}
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
					v1 + m_Position + dir * (j * density),
					0.4f, 0.05, true, true));

			//new Particle(
			//	v1 * 0.8f + m_Position + dir * (j * density),
			//	0.4f, 0.05, false, true));

			m_GhostParticles[m_GhostParticles.size() - 1]->m_LocalPosition = v1 + dir * (j * density);
		}
	}
}

float RigidBody::DistInterval(float minA, float maxA, float minB, float maxB) {
	if (minA < minB) {
		return minB - maxA;
	}
	//Else
	return minA - maxB;
}

vector<float> RigidBody::Project(Vec2f axis, float min, float max) {
	// To project a point on an axis use the dot product
	float dotProduct =  axis * (m_Vertices[0]->m_Position + m_Position);
	vector<float> result;
	min = dotProduct;
	max = dotProduct;
	for (int i = 0; i < m_Vertices.size(); i++) {
		dotProduct = (m_Vertices[i]->m_Position + m_Position) * axis;
		if (dotProduct < min) {
			min = dotProduct;
		}
		else {
			if (dotProduct > max) {
				max = dotProduct;
			}
		}
	}
	result.push_back(min);
	result.push_back(max);
	return result;
}

Vec2f RigidBody::CollisionCheck(RigidBody* other, Vec2f velocity, float dt) {
	m_Intersect = true;
	m_WillIntersect = true;
	m_MinTranslation = Vec2f(0, 0);

	int edgeCountA = m_Vertices.size();
	int edgeCountB = other->m_Vertices.size();
	float minIntervalDistance = FLT_MAX;
	Vec2f translationAxis = Vec2f();
	//Edge is starting point of the edge, the edge will be m_Vertices[i] and m_Vertices[(i + 1) % size]
	Vec2f edge;

	// Loop through all the edges of both polygons
	for (int i = 0; i < edgeCountA + edgeCountB; i++) {
		if (i < edgeCountA) {
			//Find vector from first point to the next, the 'edge'
			edge = (m_Vertices[(i + 1) % edgeCountA]->m_Position + m_Position) - (m_Vertices[i]->m_Position + m_Position);
		}
		else {
			edge = (other->m_Vertices[((i - edgeCountA) + 1) % edgeCountB]->m_Position + other->m_Position) - (other->m_Vertices[(i - edgeCountA)]->m_Position + other->m_Position);
		}

		// ===== 1. Find if the polygons are currently intersecting =====

		// Find the axis perpendicular to the current edge
		Vec2f axis = Vec2f(-edge[1], edge[0]);
		axis = Util::normalise(axis);

		// Find the projection of the polygon on the current axis
		float minA = 0; float minB = 0; float maxA = 0; float maxB = 0;
		vector<float> resultA = Project(axis, minA, maxA);
		vector<float> resultB = other->Project(axis, minB, maxB);
		minA = resultA[0];
		maxA = resultA[1];
		minB = resultB[0];
		maxB = resultB[1];
		// Check if the polygon projections are currentlty intersecting
		if (DistInterval(minA, maxA, minB, maxB) > 0)
		{
			//clear the predicted intersect points
			m_Intersect = false;
		}

		// ===== 2. Now find if the polygons *will* intersect =====

		// Project the velocity on the current axis
		float velocityProjection = axis * velocity * dt;

		// Get the projection of polygon A during the movement
		if (velocityProjection < 0) {
			minA += velocityProjection;
		}
		else {
			maxA += velocityProjection;
		}

		// Do the same test as above for the new projection
		float intervalDistance = DistInterval(minA, maxA, minB, maxB);
		if (intervalDistance > 0) m_WillIntersect = false;

		// If the polygons are not intersecting and won't intersect, exit the loop
		if (!m_Intersect && !m_WillIntersect) break;

		// Check if the current interval distance is the minimum one. If so store
		// the interval distance and the current distance.
		// This will be used to calculate the minimum translation vector
		intervalDistance = abs(intervalDistance);
		if (intervalDistance < minIntervalDistance) {
			minIntervalDistance = intervalDistance;
			translationAxis = axis;

			Vec2f d = m_Position - other->m_Position;
			if (d * translationAxis < 0)
				translationAxis = -translationAxis;
		}
	}

	// The minimum translation vector
	// can be used to push the polygons appart.
	if (m_WillIntersect)
	    m_MinTranslation = translationAxis * minIntervalDistance;

	return m_MinTranslation;
}

vector<Vec2f> RigidBody::findImpactPoint(RigidBody* other)
{
	vector<Vec2f> closestPoints;
	Vec2f e1;
	Vec2f e2;
	Vec2f p;
	for (int j = 0; j < m_Vertices.size(); j++)
	{
		p = m_Vertices[j]->m_Position + m_Position;
		bool inside = true;
		for (int i = 0; i < other->m_Vertices.size(); i++)
		{

			//Get the points of an edge of the other rigid body
			e1 = (other->m_Vertices[i]->m_Position + other->m_Position);
			e2 = (other->m_Vertices[(i + 1) % other->m_Vertices.size()]->m_Position + other->m_Position);
			
			float dist = Util::distancePointToLineSegment(e1, e2, p);
			float side = (p[0] - e1[0])*(e2[1] - e1[1]) - (p[1] - e1[1])*(e2[0] - e1[0]); //Find on which side of the line the point lies
			if (side >= 0)
				inside = false; //The point is outside on of the edges

			//The point collides with an edge of the other rigid body
			if (dist < 0.001)
			{
				bool pointFound = false;
				//Don't include equal points
				for (int k = 0; k < closestPoints.size(); k++)
				{
					if (p[0] == closestPoints[k][0] && p[1] == closestPoints[k][1]) 
					{
						pointFound = true;
						break;
					}

				}
				if (!pointFound)
				{
					closestPoints.push_back(p);
					m_ImpactPoints.push_back(p);
				}
			}
		}
		//The point lies within the other rigid body
		if (inside)
		{

			bool pointFound = false;
			//Don't include equal points
			for (int k = 0; k < closestPoints.size(); k++)
			{
				if (p[0] == closestPoints[k][0] && p[1] == closestPoints[k][1]) 
				{
					pointFound = true;
					break;
				}

			}
			if (!pointFound)
			{
				closestPoints.push_back(p);
				m_ImpactPoints.push_back(p);
			}
		}
	}


	//Vec2f checkPoint = Vec2f((m_Position[0] + other->m_Position[0]) / 2, (m_Position[1] + other->m_Position[1]) / 2);
	//float minDist = 0.15;

	//for (int i = 0; i < m_Vertices.size(); i++)
	//{
	//	Vec2f vertPosition = m_Vertices[i]->m_Position + m_Position;
	//	float currentDist = sqrt(pow(vertPosition[0] - checkPoint[0], 2) + pow(vertPosition[1] - checkPoint[1], 2));
	//	if (minDist > currentDist)
	//	{
	//		//minDist = currentDist;
	//		closestPoints.push_back(vertPosition);
	//		m_ImpactPoints.push_back(vertPosition);
	//	}
	//}
	//for (int i = 0; i < other->m_Vertices.size(); i++)
	//{
	//	Vec2f vertPosition = other->m_Vertices[i]->m_Position + other->m_Position;
	//	float currentDist = sqrt(pow(vertPosition[0] - checkPoint[0], 2) + pow(vertPosition[1] - checkPoint[1], 2));
	//	if (minDist > currentDist)
	//	{
	//		minDist = currentDist;
	//		closestPoint = vertPosition;
	//	}
	//}
	return closestPoints;
}

void RigidBody::updateGhostParticles()
{
	// Update ghost particle positions
	for (int i = 0; i < m_GhostParticles.size(); i++)
{
		matrix currentVertex = matrix(2, 1);
		currentVertex.setValue(0, 0, m_GhostParticles[i]->m_LocalPosition[0]);
		currentVertex.setValue(1, 0, m_GhostParticles[i]->m_LocalPosition[1]);
		matrix result = *m_Rotation * currentVertex;
		m_GhostParticles[i]->m_LocalPosition = Vec2f(result.getValue(0, 0), result.getValue(1, 0));
		
		// TODO: apply rotation
		m_GhostParticles[i]->m_Position = m_GhostParticles[i]->m_LocalPosition + m_Position;
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