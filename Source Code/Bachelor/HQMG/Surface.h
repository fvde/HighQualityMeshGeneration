#pragma once
/**
* @author  Ferdinand "Fred" von den Eichen
*
* License: NDL
*/

#include "Spline.h"
#include "ShardFileParser.h"
#include "niven.Core.MemoryLayout3D.h"
#include "Core/inc/Iterator3D.h"
#include "niven.Core.Exception.h"
#include "niven.Core.Math.Vector.h"
#include "niven.Core.PointerHelper.h"

#include <algorithm>

using namespace niven;

class Surface {
public:
	NIV_DEFINE_CLASS_SHARED_PTR(Surface);
public:
	enum Direction {
		X = 0,
		Y = 1,
		Z = 2
	};

	virtual float Evaluate(float x, float y, float z) = 0;
	float Evaluate(Vector3f pos);
	virtual float RaycastInDirection(float x, float y, float z, Direction d) = 0;
	virtual bool ProjectOnSurface(Vector3f& initialLocation, const Vector3f& projectionAxis, Vector3f& projectionLocation) = 0;
	virtual float EvaluateFirstOrderXDerivative(float x, float y, float z) = 0;
	virtual float EvaluateFirstOrderYDerivative(float x, float y, float z) = 0;
	virtual float EvaluateFirstOrderZDerivative(float x, float y, float z) = 0;
	virtual float EvaluateSecondOrderXDerivative(float x, float y, float z) = 0;
	virtual float EvaluateSecondOrderYDerivative(float x, float y, float z) = 0;
	virtual float EvaluateSecondOrderZDerivative(float x, float y, float z) = 0;
	virtual float EvaluateFirstXThenYDerivative(float x, float y, float z) = 0;
	virtual float EvaluateFirstXThenZDerivative(float x, float y, float z) = 0;
	virtual float EvaluateFirstYThenZDerivative(float x, float y, float z) = 0;
	float EvaluateFirstOrderXDerivative(Vector3f pos);
	float EvaluateFirstOrderYDerivative(Vector3f pos);
	float EvaluateFirstOrderZDerivative(Vector3f pos);
	float EvaluateSecondOrderXDerivative(Vector3f pos);
	float EvaluateSecondOrderYDerivative(Vector3f pos);
	float EvaluateSecondOrderZDerivative(Vector3f pos);
	float EvaluateFirstXThenYDerivative(Vector3f pos);
	float EvaluateFirstXThenZDerivative(Vector3f pos);
	float EvaluateFirstYThenZDerivative(Vector3f pos);
	virtual std::vector<std::pair<Vector3f, Spline*>> GetSurfaceSplines() = 0;
	virtual	bool ValidCoordinates(int x, int y, int z) const = 0;
	virtual void SetApproximationSteps(int counter) = 0;
	virtual void SetApproximationAccuracy(float accuracy) = 0;
	virtual Vector3f GetNormal(Vector3f pos) = 0;

};

class SplineSurface : public Surface {
	public: 
		SplineSurface();
		SplineSurface(const ShardFileParser::Ptr& sfp, float isovalue);
		float Evaluate(float x, float y, float z);
		float RaycastInDirection(float x, float y, float z, Direction d);
		bool ProjectOnSurface(Vector3f& initialLocation, const Vector3f& projectionAxis, Vector3f& projectionLocation);
		float EvaluateFirstOrderXDerivative(float x, float y, float z);
		float EvaluateFirstOrderYDerivative(float x, float y, float z);
		float EvaluateFirstOrderZDerivative(float x, float y, float z);
		float EvaluateSecondOrderXDerivative(float x, float y, float z);
		float EvaluateSecondOrderYDerivative(float x, float y, float z);
		float EvaluateSecondOrderZDerivative(float x, float y, float z);
		float EvaluateFirstXThenYDerivative(float x, float y, float z);
		float EvaluateFirstXThenZDerivative(float x, float y, float z);
		float EvaluateFirstYThenZDerivative(float x, float y, float z);
		std::vector<std::pair<Vector3f, Spline*>> GetSurfaceSplines();
		bool ValidCoordinates(int x, int y, int z) const;
		void SetApproximationSteps(int counter);
		void SetApproximationAccuracy(float accuracy);
		Vector3f GetNormal(Vector3f pos);

	private: 
		std::vector<Spline> splines_;
		float approximationAccuracy_;
		int approximationStepCounter_;
		std::vector<std::pair<Vector3f, Spline*>> isoSurfaceSplines_;
		MemoryLayout3D layout_;
		ShardFileParser::Ptr sfp_;
		float isovalue_;
};
