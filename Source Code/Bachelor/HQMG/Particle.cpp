#include "Particle.h"

Particle::Particle(float c, std::vector<Atom> a) 
	: Coefficient(c){
	for (auto it = a.begin(); it != a.end(); it++){
		this->operator*=(*it);
	}
}

Particle::Particle(float c)
	: Coefficient(c){}

Particle::Particle(float c, Atom a)
	: Coefficient(c){
		(*this) *= a;
	}

Particle::Particle()
	: Coefficient(0){}

float Particle::Evaluate(std::vector<float> values, std::vector<char> variables) {
	if( values.size() != variables.size()){
		throw niven::Exception("Number of values does not match number of variables. Can't evaluate particle.", "No details.");
	}
	
	float result = Coefficient;

	for (int x = 0; x < values.size(); x++){
		if(Atoms.find(variables[x]) != Atoms.end()){
			result *= Atoms[variables[x]].Evaluate(values[x]);
		}
	}

	return result;
}

void Particle::Derive(char variable) {
	if (Atoms.find(variable) != Atoms.end()){
		Coefficient *= Atoms[variable].Exponent;
		Atoms[variable].Exponent--;

		// Remove an atom if its exponent is 0
		if(Atoms[variable].Exponent == 0){
			Atoms.erase(variable);
		}

	} else {
		Coefficient = 0;
		Atoms.clear();
	}
}

// Return reference to myself (left side arg remains unchanged)
Particle Particle::operator+=(const Particle &other) {
	if(EqualVariablesAndExponents(other)){
		Coefficient += other.Coefficient;
	} else {
		throw niven::Exception("Cant add particles with different exponents or variables!", "No details.");
	}

	return *this;
}


// Return reference to new object (both sides remain unchanged)
Particle Particle::operator+(const Particle &other) {
	// Make a copy of myself and add.
    return Particle(*this) += other;
}

Particle Particle::operator*=(const Atom &otherAtom) {
	if(Atoms.find(otherAtom.Variable) == Atoms.end()){
		// Add new Atom
		Atoms[otherAtom.Variable] = otherAtom;
	} else {
		// Multiply
		Atoms[otherAtom.Variable] *= otherAtom;

		if(Atoms[otherAtom.Variable].Exponent == 0){
			Atoms.erase(otherAtom.Variable);
		}
	}
	return *this;
}

// Return reference to new object (both sides remain unchanged)
Particle Particle::operator*(const Atom &otherAtom) {
	// Make a copy of myself and multiply.
    return Particle(*this) *= otherAtom;
}

// For each atom in the other particle, multiply it on our particle
Particle Particle::operator*=(const Particle &otherParticle) {

	if(otherParticle.Coefficient != 0){	
		for (auto it = otherParticle.Atoms.begin(); it != otherParticle.Atoms.end(); it++){
			this->operator*=(it->second);
		}
		Coefficient *= otherParticle.Coefficient;
	} else {
		Atoms.clear();
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
	if(Atoms.size() != other.Atoms.size()){
		return false;
	} else {
		auto it2 = other.Atoms.begin();
		for(auto it1 = Atoms.begin(); it1 != Atoms.end(); it1++, it2++){
			if(*it1 != *it2){
				return false;
			}
		}
	}
	return true;
}