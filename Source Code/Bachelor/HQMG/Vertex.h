#pragma once
/**
* @author  Ferdinand "Fred" von den Eichen
*
* License: NDL
*/

#include "niven.Core.Math.Vector.h"

using namespace niven;

class Vertex {

public:
	Vertex();
	Vertex(Vector3f pos);

public:
	Vector3f Position;
	Vector3f Normal;
	int Index;
	Vector4f Color;
	float IdealEdgeLength;
};
