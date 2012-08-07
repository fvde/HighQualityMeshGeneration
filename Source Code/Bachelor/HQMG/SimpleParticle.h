#pragma once
/**
* @author  Ferdinand "Fred" von den Eichen
*
* License: NDL
*/

#include <math.h>
#include <vector>
#include <map>
#include "Atom.h"


// This version of the particle class uses a vector instead 
// of a map and does not rely on the Atom class anymore.
class Particle {

public:

	Particle(int size);
	Particle(float c, int size);
	Particle(float c, int v1, int v2, int v3);
	float Evaluate(std::vector<float> values);
	void Derive(int position);
	bool EqualVariablesAndExponents(const Particle &other) const;
	Particle operator+=(const Particle &other);
	Particle operator+(const Particle &other);
	Particle operator*=(const Particle &other);
	Particle operator*(const Particle &other);
	bool operator==(const Particle &other) const;

private:

public:

	float Coefficient;
	std::vector<int> Exponents;

private:

};