#pragma once
/**
* @author  Ferdinand "Fred" von den Eichen
*
* License: NDL
*/

#include <functional>
#include <math.h>
#include <vector>
#include "niven.Core.Math.Vector.h"

using namespace niven;

struct GuidanceFieldSample
{
		Vector3f					position;
		float						idealEdgeLength,
									curvature,
									isovalue;
		Vector4f					debugColor;
		unsigned int				childCode; // For the octree
};

struct OctreeBoundingBox
{
        Vector3f		center;         // Center of a cubic bounding volume
        double			radius;         // Radius of a cubic bounding volume
};

class Octree {

	//typedef bool (*TraversalFunction)(const Octree &o, const niven::Vector3f pos, std::vector<GuidanceFieldSample>& data);
	typedef std::function<bool (const Octree &o, const niven::Vector3f pos, float distance, std::vector<GuidanceFieldSample>& data)> TraversalFunction;

public:
	Octree();
	bool Build(std::vector<GuidanceFieldSample> samples, 
					const std::vector<niven::Vector3f>& offsetTable,
					const unsigned int count, 
					const OctreeBoundingBox &bounds);
	void Traverse(TraversalFunction& proc, const Vector3f& pos, float distance, std::vector<GuidanceFieldSample>& data) const;
	bool IsLeaf() const;
	bool PositionIsWithinDistance(const Vector3f& pos, float distance) const;
	GuidanceFieldSample GetLeafSample() const;

private:
	void calculateBoundingSphere(std::vector<GuidanceFieldSample>& samples, OctreeBoundingBox& bounds);

protected:
	unsigned int						sampleCount_;
	OctreeBoundingBox					boundingSphere_;
	Octree*								child_[8];
	GuidanceFieldSample					sample_;
	std::vector<niven::Vector3f>		boundsOffsetTable_;
};
