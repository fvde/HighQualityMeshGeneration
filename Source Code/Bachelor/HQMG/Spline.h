#pragma once
/**
* @author  Ferdinand "Fred" von den Eichen
*
* License: NDL
*/

#include <math.h>
#include <vector>
#include "MultivariatePolynom.h"
#include "niven.Core.Math.Matrix.h"
#include "niven.Core.Math.MatrixOperators.h"
#include "Core/inc/MemoryLayout.h"
#include "Core/inc/Iterator3D.h"

class Spline {

public:
	Spline();
	float Evaluate(float x, float y, float z);
	
private:

protected:
	Spline(niven::Matrix4f& m, std::vector<float>& s);

public:
	MultivariatePolynom Function;
	MultivariatePolynom FirstOrderXDerivative;
	MultivariatePolynom FirstOrderYDerivative;
	MultivariatePolynom FirstOrderZDerivative;

	MultivariatePolynom SecondOrderXDerivative;
	MultivariatePolynom SecondOrderYDerivative;
	MultivariatePolynom SecondOrderZDerivative;

	MultivariatePolynom FirstXThenYDerivative;
	MultivariatePolynom FirstXThenZDerivative;
	MultivariatePolynom FirstYThenZDerivative;
};

class BSpline : public Spline {
	public: 
		BSpline(std::vector<float>& samples);
};

class CardinalSpline : public Spline {
	public: 
		CardinalSpline(float r, std::vector<float>& samples);
};

class CatmullRomSpline : public CardinalSpline {
	public: 
		CatmullRomSpline(std::vector<float>& samples);
};


class EmptySpline : public Spline {
	public: 
		EmptySpline() {}
		float Evaluate(float x, float y, float z) { return 0.0f;}
};