#pragma once
/**
* @author  Ferdinand "Fred" von den Eichen
*
* License: NDL
*/

#include <math.h>
#include <vector>
#include "SimpleParticle.h"



class MultivariatePolynom {

public:

	MultivariatePolynom();
	MultivariatePolynom(Particle p);
	MultivariatePolynom(Particle p1, Particle p2, Particle p3, Particle p4);
	MultivariatePolynom(std::vector<Particle> particles);
	float Evaluate(std::vector<float> values);
	float Evaluate(float x, float y, float z);
	MultivariatePolynom Derive(int position);
	void CleanUp();
	MultivariatePolynom operator+=(const Particle &other);
	MultivariatePolynom operator+(const Particle &other);
	MultivariatePolynom operator+=(const MultivariatePolynom &other);
	MultivariatePolynom operator+(const MultivariatePolynom &other);
	MultivariatePolynom operator*=(const Particle &other);
	MultivariatePolynom operator*(const Particle &other);
	MultivariatePolynom operator*=(const MultivariatePolynom &other);
	MultivariatePolynom operator*(const MultivariatePolynom &other);
	MultivariatePolynom operator*=(const float other);
	MultivariatePolynom operator*(const float other);

private:

public:

private:

	std::vector<Particle> particles_;
};