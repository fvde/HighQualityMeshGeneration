#include "Vertex.h"

Vertex::Vertex()
	: Position(Vector3f(0.0f, 0.0f, 0.0f)),
	Index(-1),
	Color(1.0f, 0.0f, 0.0f, 1.0f),
	Normal(Vector3f(0.0f, 0.0f, 0.0f)),
	IdealEdgeLength(std::numeric_limits<float>::infinity())
{
}

Vertex::Vertex(Vector3f pos)
	: Position(pos),
	Index(-1),
	Color(1.0f, 0.0f, 0.0f, 1.0f),
	Normal(Vector3f(0.0f, 0.0f, 0.0f)),
	IdealEdgeLength(std::numeric_limits<float>::infinity())
{

}