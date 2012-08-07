#include "SimpleParticle.h"

Particle::Particle(float c, int v1, int v2, int v3) 
	: Coefficient(c){
		Exponents.resize(3);
		Exponents[0] = v1;
		Exponents[1] = v2;
		Exponents[2] = v3;
	}

Particle::Particle(float c, int size)
	: Coefficient(c){
		Exponents.resize(size);
	}


Particle::Particle(int size)
	: Coefficient(0){
		Exponents.resize(size);
	}

float Particle::Evaluate(std::vector<float> values) {
	if( values.size() != Exponents.size()){
		throw niven::Exception("Number of values does not match number of variables. Can't evaluate particle.", "No details.");
	}
	
	float result = Coefficient;

	for (int x = 0; x < Exponents.size(); x++){
		result *= pow(values[x], Exponents[x]);
	}

	return result;
}

// Positions are ususally 1, 2, 3 = x, y, z
void Particle::Derive(int position) {
	if (Exponents[position] != 0){
		Coefficient *= Exponents[position];
		Exponents[position]--;

	} else {
		Coefficient = 0;
	}
}

// Return reference to myself (left side arg remains unchanged). Assume for efficiency reasons that users will call EqualVariablesAndExponents before attempting to add
Particle Particle::operator+=(const Particle &other) {
	Coefficient += other.Coefficient;
	return *this;
}


// Return reference to new object (both sides remain unchanged)
Particle Particle::operator+(const Particle &other) {
	// Make a copy of myself and add.
    return Particle(*this) += other;
}

// For each atom in the other particle, multiply it on our particle
Particle Particle::operator*=(const Particle &otherParticle) {

	if(otherParticle.Coefficient != 0){	
		for (int x = 0; x < Exponents.size(); x++){
			Exponents[x] += otherParticle.Exponents[x];
		}
		Coefficient *= otherParticle.Coefficient;
	} else {
		Coefficient = 0;
	}
	return *this;
}

// Return reference to new object (both sides remain unchanged)
Particle Particle::operator*(const Particle &otherParticle) {
	// Make a copy of myself and multiply.
    return Particle(*this) *= otherParticle;
}

bool Particle::operator==(const Particle &other) const {
	return EqualVariablesAndExponents(other) && Coefficient == other.Coefficient;
}

bool Particle::EqualVariablesAndExponents(const Particle &other) const {
	if(Exponents.size() != other.Exponents.size()){
		return false;
	} else {
		for (int x = 0; x < Exponents.size(); x++){
			if(Exponents[x] != other.Exponents[x]){
				return false;
			}
		}
	}
	return true;
}