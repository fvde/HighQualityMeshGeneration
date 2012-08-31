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
#include "niven.Core.MemoryLayout3D.h"

#include "niven.Core.IO.Path.h"
#include "ShardFileParser.h"
#include "Surface.h"
#include "GuidanceField.h"
#include "niven.Core.Color.h"
#include "niven.Image.ColorUtility.h"

#include "Core/inc/Timer.h"
#include "niven.Core.Math.Vector.h"
#include "Front.h"
#include <boost/math/special_functions/fpclassify.hpp>

using namespace niven;

class MeshExtractor {

public:

	struct Triangle 
	{
		Vertex v1, v2, v3;
	};

	struct Edge {

		Edge() {

		}
		
		Edge(Front::FrontListIterator it1, Front::FrontListIterator it2) {
			if (it1->front != it2->front){
				//throw Exception();
			}

			frontIterator1 = it1;
			frontIterator2 = it2;
			v1 = it1->vertex.Position;
			v2 = it2->vertex.Position;
			front = it1->front;
		}

		Edge(Vertex v, Front::FrontListIterator it2) {
			frontIterator2 = it2;
			v1 = v.Position;
			v2 = it2->vertex.Position;
			front = it2->front;
		}
		

		Vector3f v1, v2;
		Front::FrontListIterator frontIterator1;
		Front::FrontListIterator frontIterator2;
		int front;
	};

	struct Intersection {
		MeshExtractor::Edge edge;
		Front::FrontListIterator origin;
		float x, y;
		bool insideTriangle;
	};

	MeshExtractor(){};
	MeshExtractor(const ShardFileParser::Ptr& sfp, float isoValue, float p, float n);
	void Setup();
	void TriangulateShardFile(std::vector<Vector3f>& vertices, std::vector<Vector3f>& normals, std::vector<Vector4f>& colors, std::vector<uint32>& indices, int steps);
	bool CheckFrontInterference(Vertex newVertex, Front::FrontListIterator& bestOrigin, Front::FrontListIterator& bestNeighbor, Front::FrontListIterator& bestIntersection, bool& interferenceTestFailed);
	bool GrowVertex(Vertex& v1, Vertex& v2, Vertex& newVertex);
	bool ProjectVertexOnSurface(Vertex& v);
	Triangle CreateTriangle(Vertex& v1, Vertex& v2, Vertex& v3);
	Triangle CreateTriangle(Front& f);
	bool GetIntersection(const MeshExtractor::Edge& e1, const MeshExtractor::Edge& e2, MeshExtractor::Intersection& intersection);
	static bool AngleIsValid(const Vector2f& origin, const Vector2f& dest, const Vector2f& point);

private:
	std::vector<MeshExtractor::Edge> GetEdgesInRange(const Vertex& v, const MeshExtractor::Edge& edgeCurrentPrevious);
	void GetFrontFromSeed(Vector3f seed, FrontManager& fm);
	void TransformEdge(MeshExtractor::Edge& edge, const Vector3f& translation, const Matrix3f& transformation);
	bool FindBestIntersection(const std::vector<MeshExtractor::Intersection>& intersections, const MeshExtractor::Edge& originalEdge, Front::FrontListIterator& bestOrigin, Front::FrontListIterator& bestNeighbor, Front::FrontListIterator& bestIntersection);
	void ParseFrontManager();
	bool TestZCoordinate(const MeshExtractor::Edge& edge, const float acceptance);
	void TestEdge(const MeshExtractor::Edge& edge, const std::vector<MeshExtractor::Edge>& edgesInRange, std::vector<MeshExtractor::Intersection>& intersections);
	void FindPointWithinTriangle(const MeshExtractor::Intersection& intersection, const Vector2f& v1, const Vector2f& v2, const Vector2f& v3, std::vector<MeshExtractor::Intersection>& intersections);
	bool IsInsideTriangle(const Vector2f& a, const Vector2f& b, const Vector2f& c, const Vector2f& point);
	bool IsSame2DVertex(const Vector2f& v1, const Vector2f& v2);
	bool IsBetterIntersection(const MeshExtractor::Intersection& intersection, const float& distance, const float& insideTriangleMinimum, const float& splitMinimum, const float& overallMinimum, const int& originalFront);
	bool FindBestIntersectionElement(const MeshExtractor::Intersection& intersection, const MeshExtractor::Edge& originalEdge, Front::FrontListIterator& bestElement);
	void TestTriangle(MeshExtractor::Edge& intersectionOrigin, MeshExtractor::Edge& intersectionNeighbor, MeshExtractor::Edge& originNeighbor, const Vertex& newVertex, const Vector3f& translation, const Matrix3f& transformation, std::vector<MeshExtractor::Intersection>& intersections, const bool& isSecondaryTest);
	void Get2DTransformation(const MeshExtractor::Edge& edge1, const MeshExtractor::Edge& edge2, Vector3f& translation, Matrix3f& transformation);
	void DisplayProgress(const int& current, const int& total);

public:
	std::vector<std::vector<Vector3f>> DebugFront;
	Vector3f PreOrigin;
	Vector3f PreNeighbor;
	Vector3f PreAttempt;
	Vector3f PostOrigin;
	Vector3f PostNeighbor;
	Vector3f PostAttempt;

	Vector3f TransformedPreOrigin;
	Vector3f TransformedPreNeighbor;
	Vector3f TransformedPreAttempt;

	Vector3f TransformedPostOrigin;
	Vector3f TransformedPostNeighbor;
	Vector3f TransformedPostAttempt;

	std::vector<Edge> LastEdgesInRange;
	std::vector<Vector3f> Intersections;
	std::vector<Vector3f> AdditionalDebugInfo;

private:
	ShardFileParser::Ptr sfp_;
	FrontManager frontManager_;
	float isoValue_;
	GuidanceField::Ptr guidanceField_;
	Surface::Ptr surface_;
	int indiceCounter_;
	int	debugMaxTriangleNumber_;
	int debugTriangleCounter_;
	int totalSteps_;
	float floatingPointPrecision_;
	float collisionZCoordinateAcceptance_;
	bool debugMode_;
	Vector4f meshColor_;
};