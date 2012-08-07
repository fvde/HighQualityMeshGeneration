#include "Atom.h"

Atom::Atom(char v)
	: Exponent(1),
	Variable(v){

}

Atom::Atom(char v, int e)
	: Exponent(e),
	Variable(v){
}

// Default constructor for map
Atom::Atom(){
	Exponent = 0;
	Variable = '@';
}

float Atom::Evaluate(float value) const {
	return pow(value, Exponent);
}



// Return reference to myself (left side arg remains unchanged)
Atom Atom::operator*=(const Atom &other) {
	if(Variable == other.Variable){
		Exponent += other.Exponent;
	} else {
		throw niven::Exception("Cant multiply atoms with different variables!", "No details.");
	}
	return *this;
}

// Return reference to new object (both sides remain unchanged)
Atom Atom::operator*(const Atom &other) {
	// Make a copy of myself.  Same as MyClass result(*this);
	return Atom(*this) *= other;
}

bool Atom::operator==(const Atom &other) const {
	return (Exponent == other.Exponent && Variable == other.Variable);
}

bool Atom::operator!=(const Atom &other) const {
	return !(*this == other);
}

bool Atom::operator<(const Atom &other) const {
	return Variable < other.Variable;
}