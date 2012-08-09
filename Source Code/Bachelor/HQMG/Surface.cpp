#include "Surface.h"

SplineSurface::SplineSurface() {}

float Surface::Evaluate(Vector3f pos) {
	return Evaluate(pos.X(), pos.Y(), pos.Z());
}

float Surface::EvaluateFirstOrderXDerivative(Vector3f pos) {
	return EvaluateFirstOrderXDerivative(pos.X(), pos.Y(), pos.Z());
}

float Surface::EvaluateFirstOrderYDerivative(Vector3f pos) {
	return EvaluateFirstOrderYDerivative(pos.X(), pos.Y(), pos.Z());
}

float Surface::EvaluateFirstOrderZDerivative(Vector3f pos) {
	return EvaluateFirstOrderZDerivative(pos.X(), pos.Y(), pos.Z());
}

float Surface::EvaluateSecondOrderXDerivative(Vector3f pos) {
	return EvaluateSecondOrderXDerivative(pos.X(), pos.Y(), pos.Z());
}

float Surface::EvaluateSecondOrderYDerivative(Vector3f pos) {
	return EvaluateSecondOrderYDerivative(pos.X(), pos.Y(), pos.Z());
}

float Surface::EvaluateSecondOrderZDerivative(Vector3f pos) {
	return EvaluateSecondOrderZDerivative(pos.X(), pos.Y(), pos.Z());
}

float Surface::EvaluateFirstXThenYDerivative(Vector3f pos) {
	return EvaluateFirstXThenYDerivative(pos.X(), pos.Y(), pos.Z());
}

float Surface::EvaluateFirstXThenZDerivative(Vector3f pos) {
	return EvaluateFirstXThenZDerivative(pos.X(), pos.Y(), pos.Z());
}

float Surface::EvaluateFirstYThenZDerivative(Vector3f pos) {
	return EvaluateFirstYThenZDerivative(pos.X(), pos.Y(), pos.Z());
}

SplineSurface::SplineSurface(const ShardFileParser::Ptr& sfp, float isovalue)
	: sfp_(sfp),
	isovalue_(isovalue){

		Log::Debug("Surface", "Starting surface calculation...");
	
		// Default values for projection accuracy
		approximationAccuracy_ = 0.02f;
		approximationStepCounter_ = 20;

		layout_ = MemoryLayout3D(sfp->getVolumeWidth());
		splines_ = std::vector<Spline>(layout_.GetStorageSize());

		Iterator3D it (layout_);

		// Outer samples
		std::vector<float> samples (64);
		MemoryLayout3D samplelayout (4);
		Iterator3D sampleIterator (samplelayout);

		// Inner samples for each cell
		std::vector<float> innerSamples (8);
		MemoryLayout3D innerSamplelayout (2);
		Iterator3D innerSampleIterator (innerSamplelayout);

		while (!it.IsAtEnd()) {

			// Find samples in a 4 x 4 area
			sampleIterator.Reset();

			while(!sampleIterator.IsAtEnd())
			{
				samples[samplelayout (sampleIterator)] = sfp->GetDensity(	it.X() + sampleIterator.X() - 1,
																			it.Y() + sampleIterator.Y() - 1,
																			it.Z() + sampleIterator.Z() - 1);
				sampleIterator++;
			}

			// For each cell decide if the iso-surface passes through
			// This is the case if of the 8 inner values maximum and minimum include the iso-value
			innerSampleIterator.Reset();
			while(!innerSampleIterator.IsAtEnd())
			{
				innerSamples[innerSamplelayout (innerSampleIterator)]  = samples[samplelayout(innerSampleIterator.X() + 1, innerSampleIterator.Y() + 1, innerSampleIterator.Z() + 1)];
				innerSampleIterator++;
			}

			if(	*std::min_element(innerSamples.begin(), innerSamples.begin() + innerSamples.size()) <= isovalue_
				&& *std::max_element(innerSamples.begin(), innerSamples.begin() + innerSamples.size()) >= isovalue_){
				// Create a spline, in this case CatmullRom
				splines_[layout_(it)] = CatmullRomSpline(samples);
				isoSurfaceSplines_.push_back(std::pair<Vector3i, Spline*>(it.ToVector(), &splines_[layout_(it)]));
			} else {
				splines_[layout_(it)] = EmptySpline();
			}

			it++;
		}

		Log::Debug("Surface", "Surface calculation completed!");
}

float SplineSurface::Evaluate(float x, float y, float z) {
	if(ValidCoordinates((int)x, (int)y, (int)z)){
		return splines_[layout_(int(x),int(y),int(z))].Evaluate(x - (int)x, y - (int)y, z - (int)z);
	} else {
		return 0.0f;
	}
}

float SplineSurface::EvaluateFirstOrderXDerivative(float x, float y, float z) {
	if(ValidCoordinates((int)x, (int)y, (int)z)){
		return splines_[layout_(int(x),int(y),int(z))].FirstOrderXDerivative.Evaluate(x - (int)x, y - (int)y, z - (int)z);
	} else {
		return 0.0f;
	}
}

float SplineSurface::EvaluateFirstOrderYDerivative(float x, float y, float z) {
	if(ValidCoordinates((int)x, (int)y, (int)z)){
		return splines_[layout_(int(x),int(y),int(z))].FirstOrderYDerivative.Evaluate(x - (int)x, y - (int)y, z - (int)z);
	} else {
		return 0.0f;
	}
}

float SplineSurface::EvaluateFirstOrderZDerivative(float x, float y, float z) {
	if(ValidCoordinates((int)x, (int)y, (int)z)){
		return splines_[layout_(int(x),int(y),int(z))].FirstOrderZDerivative.Evaluate(x - (int)x, y - (int)y, z - (int)z);
	} else {
		return 0.0f;
	}
}

float SplineSurface::EvaluateSecondOrderXDerivative(float x, float y, float z) {
	if(ValidCoordinates((int)x, (int)y, (int)z)){
		return splines_[layout_(int(x),int(y),int(z))].SecondOrderXDerivative.Evaluate(x - (int)x, y - (int)y, z - (int)z);
	} else {
		return 0.0f;
	}
}

float SplineSurface::EvaluateSecondOrderYDerivative(float x, float y, float z) {
	if(ValidCoordinates((int)x, (int)y, (int)z)){
		return splines_[layout_(int(x),int(y),int(z))].SecondOrderYDerivative.Evaluate(x - (int)x, y - (int)y, z - (int)z);
	} else {
		return 0.0f;
	}
}

float SplineSurface::EvaluateSecondOrderZDerivative(float x, float y, float z) {
	if(ValidCoordinates((int)x, (int)y, (int)z)){
		return splines_[layout_(int(x),int(y),int(z))].SecondOrderZDerivative.Evaluate(x - (int)x, y - (int)y, z - (int)z);
	} else {
		return 0.0f;
	}
}

float SplineSurface::EvaluateFirstXThenYDerivative(float x, float y, float z) {
	if(ValidCoordinates((int)x, (int)y, (int)z)){
		return splines_[layout_(int(x),int(y),int(z))].FirstXThenYDerivative.Evaluate(x - (int)x, y - (int)y, z - (int)z);
	} else {
		return 0.0f;
	}
}

float SplineSurface::EvaluateFirstXThenZDerivative(float x, float y, float z) {
	if(ValidCoordinates((int)x, (int)y, (int)z)){
		return splines_[layout_(int(x),int(y),int(z))].FirstXThenZDerivative.Evaluate(x - (int)x, y - (int)y, z - (int)z);
	} else {
		return 0.0f;
	}
}

float SplineSurface::EvaluateFirstYThenZDerivative(float x, float y, float z) {
	if(ValidCoordinates((int)x, (int)y, (int)z)){
		return splines_[layout_(int(x),int(y),int(z))].FirstYThenZDerivative.Evaluate(x - (int)x, y - (int)y, z - (int)z);
	} else {
		return 0.0f;
	}
}

// Returns the z component of the raycast intersection
float SplineSurface::RaycastInDirection(float x, float y, float z, Direction d) {
	// Find the raycast intersection using newton iteration
	// TODO Enter x and y and save the polynom without them

	// Hack
	float resultValue;

	switch (d) {
		case X: resultValue = x; break;
		case Y: resultValue = y; break;
		case Z: resultValue = z; break;
	}
	float previousValue = std::numeric_limits<float>::infinity();

	// To prevent endless loops
	int accuracyCounter = approximationStepCounter_;

	// Newton iteration
	while (std::abs(previousValue - resultValue) > approximationAccuracy_ && accuracyCounter > 0){
		previousValue = resultValue;

		switch (d) {
			case X: resultValue -= (Evaluate(resultValue, y, z) - isovalue_) / EvaluateFirstOrderXDerivative(resultValue, y, z); break;
			case Y: resultValue -= (Evaluate(x, resultValue, z) - isovalue_) / EvaluateFirstOrderYDerivative(x, resultValue, z); break;
			case Z: resultValue -= (Evaluate(x, y, resultValue) - isovalue_) / EvaluateFirstOrderZDerivative(x, y, resultValue); break;
		}	
		accuracyCounter--;

		// Entered a cell where the surface doesn't pass through
		if(resultValue == std::numeric_limits<float>::infinity()){
			return resultValue;
		}
	}

	if (accuracyCounter > 0){
		return resultValue;
	} else {
		return std::numeric_limits<float>::infinity();
	}
}

std::vector<std::pair<Vector3f, Spline*>> SplineSurface::GetSurfaceSplines() {
	return isoSurfaceSplines_;
}

bool SplineSurface::ValidCoordinates(int x, int y, int z) const {
	return (x < layout_.GetWidth() && 
		y < layout_.GetDepth() && 
		z < layout_.GetHeight()&& 
		x >= 0 && 
		y >= 0 && 
		z >= 0);
}

void SplineSurface::SetApproximationAccuracy(float accuracy) {
	approximationAccuracy_ = accuracy;
}

void SplineSurface::SetApproximationSteps(int counter) {
	approximationStepCounter_ = counter;
}

Vector3f SplineSurface::GetNormal(Vector3f pos){
	Vector3f n = Vector3f(Surface::EvaluateFirstOrderXDerivative(pos),
							Surface::EvaluateFirstOrderYDerivative(pos),
							Surface::EvaluateFirstOrderZDerivative(pos));
	n.Normalize();
	return n;
}