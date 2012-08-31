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
#include "niven.Core.Math.ArrayFunctions.h"
#include "niven.Core.Math.VectorFunctions.h"
#include "niven.Core.Math.MatrixFunctions.h"

#include "Volume/inc/MarchingCubes.h"
#include "niven.Core.IO.Path.h"
#include "ShardFileParser.h"

#include "Core/inc/Timer.h"
#include "MultivariatePolynom.h"
#include "Surface.h"
#include "Octree.h"
#include "niven.Core.Generic.ScopedArray.h"
#include "niven.Core.PointerHelper.h"

#include <functional>

using namespace niven;

class GuidanceField {
public:
	NIV_DEFINE_CLASS_SHARED_PTR(GuidanceField);

public:
	GuidanceField(const ShardFileParser::Ptr& sfp, Surface::Ptr s, float p, float n);
	float GetDensity(float x, float y, float z) const;
	std::vector<GuidanceFieldSample> GetSamples() const;
	float Evaluate(const Vector3f pos);
	float G(const GuidanceFieldSample s, const Vector3f x);

private:

	Surface::Ptr surface_;
	MemoryLayout3D layout_;
	ShardFileParser::Ptr sfp_;
	std::vector<GuidanceFieldSample> samples_;
	float p_;
	float n_;
	int samplingDensity_;
	int randomSeed_;
	float maxTriangleLength_;
	// Octree traversal function
	std::function<bool (const Octree &o, const niven::Vector3f pos, float distance, std::vector<GuidanceFieldSample>& data)> getSamplesByDistance_;
	Octree octree_;
	float guidanceFieldEvaluationrange_;
};