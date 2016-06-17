#include "Box.h"
#include "Utility.h"


Box::Box(const Vec2f constructPos, float density, float width, float height, bool isFixed) : RigidBody(constructPos)
{
	m_Width = width;
	m_Height = height;
	m_isFixed = isFixed;
	m_Torque = 0;
	Vec2f p = constructPos;

	// Add vertices
	m_Vertices = vector<Particle*>();
	//m_Vertices.push_back(new Particle(Vec2f(p[0] - width / 2, p[1] - height / 2)));
	//m_Vertices.push_back(new Particle(Vec2f(p[0] + width / 2, p[1] - height / 2)));
	//m_Vertices.push_back(new Particle(Vec2f(p[0] + width / 2, p[1] + height / 2)));
	//m_Vertices.push_back(new Particle(Vec2f(p[0] - width / 2, p[1] + height / 2)));
	m_Vertices.push_back(new Particle(Vec2f(- width / 2, - height / 2)));
	m_Vertices.push_back(new Particle(Vec2f(width / 2, - height / 2)));
	m_Vertices.push_back(new Particle(Vec2f(width / 2, height / 2)));
	m_Vertices.push_back(new Particle(Vec2f(- width / 2, height / 2)));

	m_Mass = density * width * height;
	reset();
	calculateInertia();

}

void Box::calculateInertia()
{
	m_Inertia = m_Mass * (pow(m_Width, 2) + pow(m_Height, 2));
}

vector<float> Box::getExtremes()
{
	//Extremes in world coordinates
	vector<float> extremes = vector<float>();
	extremes.push_back(-1.0f);
	extremes.push_back(-1.0f);
	extremes.push_back(1.0f);
	extremes.push_back(1.0f);
	for (int i = 0; i < m_Vertices.size(); i++)
	{
		matrix currentVertex = matrix(2, 1);
		currentVertex.setValue(0, 0, /*rigidBodies[i]->m_Position[0]*/ -m_Vertices[i]->m_Position[0]);
		currentVertex.setValue(1, 0, /*rigidBodies[i]->m_Position[1]*/ -m_Vertices[i]->m_Position[1]);
		//currentVertex.printMatrix();
		//cout << endl;
		matrix result = *m_Rotation * currentVertex;
		if (m_Vertices[i]->m_Position[0] + m_Position[0] > extremes[0])
			extremes[0] = m_Vertices[i]->m_Position[0] + m_Position[0];
		if (m_Vertices[i]->m_Position[1] + m_Position[1] > extremes[1])
			extremes[1] = m_Vertices[i]->m_Position[1] + m_Position[1];
		if (m_Vertices[i]->m_Position[0] + m_Position[0] < extremes[2])
			extremes[2] = m_Vertices[i]->m_Position[0] + m_Position[0];
		if (m_Vertices[i]->m_Position[1] + m_Position[1] < extremes[3])
			extremes[3] = m_Vertices[i]->m_Position[1] + m_Position[1];
		currentVertex.release();
		result.release();
	}
	return extremes;
}
Vec2f Box::NarrowPhase(RigidBody* other)
{
		//updateRotation(-m_Orientation);
		//vector<float> extremes = getExtremes();

		//for (int i = 0; i < other->m_Vertices.size(); i++)
		//{
		//	//Rotate other body to current bodies
		//	matrix currentVertex = matrix(2, 1);
		//	currentVertex.setValue(0, 0, -other->m_Vertices[i]->m_Position[0] - other->m_Position[0] + m_Position[0]);
		//	currentVertex.setValue(1, 0, -other->m_Vertices[i]->m_Position[1] - other->m_Position[1] + m_Position[1]);
		//	matrix result = *m_Rotation * currentVertex;
		//	float resultx = result.getValue(0, 0);
		//	float resulty = result.getValue(1, 0);
		//	if (extremes[0] > (result.getValue(0, 0))
		//		&& extremes[1] > (result.getValue(1, 0))
		//		&& extremes[2] < (result.getValue(0, 0))
		//		&& extremes[3] < (result.getValue(1, 0)))
		//	{
		//		return other->m_Vertices[i]->m_Position + other->m_Position;
		//	}
		//	currentVertex.release();
		//	result.release();
		//}
		//updateRotation(m_Orientation);
		//return Vec2f(0, 0);
	float lowestDist = 1.00f;
	Vec2f collisionPoint = Vec2f(0, 0);
	for (int i = 0; i < m_Vertices.size(); i++)
	{
		for (int j = 0; j < other->m_Vertices.size(); j++)
		{
			Vec2f currentV = m_Vertices[i %  m_Vertices.size()]->m_Position + m_Position;
			Vec2f nextV = m_Vertices[(i + 1) % m_Vertices.size()]->m_Position + m_Position;
			Vec2f currentNorm = Vec2f((nextV[1] - currentV[1]), -(nextV[0] - currentV[0]));
			currentNorm = Util::normalise(currentNorm);
			float dist = abs(((nextV[1] - currentV[1])* (other->m_Vertices[j]->m_Position[0] + other->m_Position[0])) - ((nextV[0] - currentV[0])*(other->m_Vertices[j]->m_Position[1] + other->m_Position[1])) + nextV[0] * currentV[1] - nextV[1] * currentV[0]);
			dist = dist / sqrt(pow(nextV[1] - currentV[1], 2) + pow(nextV[0] - currentV[0], 2));
			if (dist < lowestDist)
			{
				lowestDist = dist;
				collisionPoint = other->m_Vertices[j]->m_Position + other->m_Position + currentNorm * dist;
				//normal = currentNorm;
			}
		}
	}
	return collisionPoint;
}

Box::~Box()
{
}
