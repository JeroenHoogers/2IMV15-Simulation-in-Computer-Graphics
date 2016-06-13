#pragma once
#include <gfx/vec2.h>

class Kernels
{
public:

	static double getWPoly6(float r, float h)
	{
		// Default kernel function
		// (h^2 - r^2)^3 iff 0 <= r <= h
		// otherwise 0

		double r2 = r * r;
		if (r <= h * h)
			return 4.0 / (M_PI * pow(h, 8)) * pow(h * h - r2, 3);
		else
			return 0;
	}

	static Vec2f getWSpikyGrad(Vec2f r, float h)
	{
		// Default kernel function
		// -30 / (PI * h^5) * (h-l)^2 * (r/|r|)
		// otherwise 0
		// Calculate length 
		float l = sqrt(r[0] * r[0] + r[1] * r[1]);
		if (l > 0)
			r = r / l;	// Normalize r

		if (0 <= l && l <= h)
			return -30.0f / ((float)M_PI * pow(h, 5)) * (h - l) * (h - l) * r;
		else
			return Vec2f(0, 0);
	}

	static Vec2f getWViscosityLaplace(float r, float h)
	{
		// Gradient of the kernel function
		// -12(h^2 - r^2)^2 iff 0 <= r <= h
		// otherwise 0
		if (0 <= r && r <= h)
			return (45.0f / (M_PI * pow(h, 6))) * (h - r);
		else
			return 0;
	}
};