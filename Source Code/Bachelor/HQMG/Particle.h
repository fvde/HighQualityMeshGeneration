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


class Particle {

public:

	Particle();
	Particle(float c);
	Particle(float c, Atom a);
	Particle(float c, std::vector<Atom> a);
	float Evaluate(std::vector<float> values, std::vector<char> variables);
	void Derive(char variable);
	bool EqualVariablesAndExponents(const Particle &other) const;
	Particle operator+=(const Particle &other);
	Particle operator+(const Particle &other);
	Particle operator*=(const Particle &other);
	Particle operator*(const Particle &other);
	Particle operator*=(const Atom &other);
	Particle operator*(const Atom &other);
	bool operator==(const Particle &other) const;

private:

public:

	float Coefficient;
	//std::vector<Atom> Atoms;
	std::map<char, Atom> Atoms;

private:

};