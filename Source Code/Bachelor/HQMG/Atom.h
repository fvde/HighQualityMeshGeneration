#pragma once
/**
* @author  Ferdinand "Fred" von den Eichen
*
* License: NDL
*/
#include "Core/inc/Exception.h"
#include "Core/inc/nString.h"
#include <math.h>
#include <exception>

class Atom {

public:
	Atom(char v, int e);
	Atom(char v);
	Atom();
	float Evaluate(float value) const;
	bool operator==(const Atom &other) const;
	bool operator!=(const Atom &other) const;
	bool operator<(const Atom &other) const;
	Atom operator*=(const Atom &other);
	Atom operator*(const Atom &other);

private:

public:

	int Exponent;
	char Variable;

private:

};