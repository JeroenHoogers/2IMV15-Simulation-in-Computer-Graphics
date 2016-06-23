#pragma once
#include <gfx/vec2.h>

class Util
{
public:

	// Cross w/ 2 vectors, returns a scalar
	static float crossProduct(const Vec2f a, const Vec2f b)
	{
		return a[0] * b[1] - a[1] * b[0];
	}

	// Cross product w/ vector and scalar, returns a vector
	static Vec2f crossProduct(const Vec2f a, float s)
	{
		return Vec2f( s * a[1], -s * a[0]);
	}

	// Cross product w/ scalar and vector, returns a vector
	static Vec2f crossProduct(float s, const Vec2f a)
	{
		return Vec2f(-s * a[1], s * a[0]);
	}

	static void insertionSort()
	{

	}

	static Vec2f normalise(const Vec2f a)
	{
		if (a[0] == 0 && a[1] == 0)
			return a;
		return (a / sqrt(pow(a[0], 2) + pow(a[1], 2)));
	}

	static float distancePointToLine(const Vec2f v1, const Vec2f v2, const Vec2f p)
	{
		//Absolute sign left out to know postion from line; |a|
		float a = (v2[1] - v1[1]) * p[0] - (v2[0] - v2[1]) * p[1] + v2[0] * v1[1] - v2[1] * v1[0];
		float distance = a / sqrt(pow(v2[1] - v1[1], 2) + pow(v2[0] - v1[0], 2));

		return distance;
	}
};