#include "Spline.h"

using namespace niven;
BSpline::BSpline(std::vector<float>& samples) 
	: Spline( (1.0f/6.0f) * Matrix4f(	-1.0f,	3.0f,	-3.0f,	1.0f,
						3.0f,	-6.0f,	3.0f,	0.0f,
						-3.0f,	0.0f,	3.0f,	0.0f,
						1.0f,	4.0f,	1.0f,	0.0f), samples)
{
	
}

CardinalSpline::CardinalSpline(float r, std::vector<float>& samples) 
	: Spline(Matrix4f(	-r,		2-r,	r-2,	r,
						2*r,	r-3,	3-2*r,	-r,
						-r,		0.0f,	r,		0.0f,
						0.0f,	1.0f,	0.0f,	0.0f),samples)
{
	
}

CatmullRomSpline::CatmullRomSpline(std::vector<float>& samples)
	: CardinalSpline(0.5f, samples)
{

}

// Default constructor
Spline::Spline() {}

Spline::Spline(niven::Matrix4f& m, std::vector<float>& samples) 
{
	// Do not change. M's will not adapt to this.
	int sampleSize = 4;

	std::vector<MultivariatePolynom> subfunctionsX(sampleSize);
	std::vector<MultivariatePolynom> subfunctionsY(sampleSize);
	std::vector<MultivariatePolynom> subfunctionsZ(sampleSize);

	// Initialize spline subfunctions in x-, y- and z-direction
	for (int i = 0; i < sampleSize; i++){

		subfunctionsX[i] = MultivariatePolynom(Particle(m[i], 3, 0, 0), 
										Particle(m[i + 4], 2, 0, 0), 
										Particle(m[i + 8], 1, 0, 0), 
										Particle(m[i + 12], 3));

		subfunctionsY[i] = MultivariatePolynom(Particle(m[i], 0, 3, 0), 
										Particle(m[i + 4], 0, 2, 0), 
										Particle(m[i + 8], 0, 1, 0), 
										Particle(m[i + 12], 3));

		subfunctionsZ[i] = MultivariatePolynom(Particle(m[i], 0, 0, 3), 
										Particle(m[i + 4], 0, 0, 2), 
										Particle(m[i + 8], 0, 0, 1), 
										Particle(m[i + 12], 3));
	}

	// Construct the spline from the subfunctions
	Function = MultivariatePolynom();
	MemoryLayout3D sampleLayout = MemoryLayout3D(sampleSize);
	Iterator3D it3d (sampleLayout);

	while (!it3d.IsAtEnd()){
		Function +=	subfunctionsX[it3d.X()] 
					* subfunctionsY[it3d.Y()]
					* subfunctionsZ[it3d.Z()]
					* samples[sampleLayout(it3d)];

		it3d++;
	}

	FirstOrderXDerivative = Function.Derive(0);
	FirstOrderYDerivative = Function.Derive(1);
	FirstOrderZDerivative = Function.Derive(2);

	SecondOrderXDerivative = FirstOrderXDerivative.Derive(0);
	SecondOrderYDerivative = FirstOrderYDerivative.Derive(1);
	SecondOrderZDerivative = FirstOrderZDerivative.Derive(2);

	FirstXThenYDerivative = FirstOrderXDerivative.Derive(1);
	FirstXThenZDerivative = FirstOrderXDerivative.Derive(2);
	FirstYThenZDerivative = FirstOrderYDerivative.Derive(2);
}

float Spline::Evaluate(float x, float y, float z){
	return Function.Evaluate(x, y, z);
}