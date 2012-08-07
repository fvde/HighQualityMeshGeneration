#pragma once
/**
* @author  Ferdinand "Fred" von den Eichen
*
* License: NDL
*/

#include "Core/inc/Core.h"
#include "Core/inc/CommandlineParser.h"
#include "Core/inc/Exception.h"
#include "Core/inc/io/File.h"
#include "Core/inc/io/FileSystem.h"
#include "Core/inc/Log.h"
#include "Core/inc/nString.h"
#include "Core/inc/NumericCast.h"
#include "Core/inc/StringFormat.h"
#include "Core/inc/Iterator3D.h"
#include "Core/inc/math/3dUtility.h"
#include "Core/inc/math/VectorToString.h"
#include "Core/inc/MemoryLayout.h"

#include "niven.Core.IO.Path.h"
#include "ShardFileParser.h"
#include "Surface.h"
#include "GuidanceField.h"
#include "niven.Core.Color.h"
#include "niven.Image.ColorUtility.h"

#include "Core/inc/Timer.h"
#include "niven.Core.Math.Vector.h"
#include "Front.h"

using namespace niven;

class MeshExtractor {

public:

	struct Triangle 
	{
		Vertex v1, v2, v3;
	};

	struct Edge {
		Vector3f v1, v2;
		Front::FrontListIterator frontIterator1;
		Front::FrontListIterator frontIterator2;
		int front;
	};

	struct Intersection {
		Front::FrontListIterator frontIterator;
		float x, y;
	};

	MeshExtractor(const ShardFileParser::Ptr& sfp, float isoValue, float p, float n);
	void TriangulateShardFile(std::vector<Vector3f>& vertices, std::vector<Vector3f>& normals, std::vector<Vector4f>& colors, std::vector<uint32>& indices);
	bool CheckFrontInterference(Front& current, Vertex newVertex, Front::FrontListIterator& currentFrontElementIterator, Front::FrontListIterator& interferingFrontElementIterator, bool& selfIntersection);
	bool GrowVertex(Vertex& v1, Vertex& v2, Vertex& newVertex);
	bool ProjectVertexOnSurface(Vertex& v);
	Triangle CreateTriangle(Vertex v1, Vertex v2, Vertex v3);
	bool GetIntersection(const MeshExtractor::Edge& e1, const MeshExtractor::Edge& e2, MeshExtractor::Intersection& intersection);

private:
	std::vector<MeshExtractor::Edge> GetEdgesInRange(Vertex& v, const Vector3f& v1, const Vector3f& v2);
	void GetFrontFromSeed(Vector3f seed, FrontManager& fm);
	void TransformEdge(MeshExtractor::Edge& edge, const Vector3f& translation, const Matrix3f& transformation);
	bool IsCloseSelfIntersection(Front::FrontListIterator& interfering, Front::FrontListIterator& current);
	bool GiveDebugFeedback(int& debugTriangleCounter, int max);

private:
	ShardFileParser::Ptr sfp_;
	FrontManager frontManager_;
	float isoValue_;
	GuidanceField::Ptr guidanceField_;
	Surface::Ptr surface_;
};