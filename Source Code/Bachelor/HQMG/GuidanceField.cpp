#include "GuidanceField.h"



GuidanceField::GuidanceField(const ShardFileParser::Ptr& sfp, Surface::Ptr s, float p, float n)
	:	sfp_(sfp),
		p_(p),
		n_(n),
		surface_(s){

		Log::Info("GuidanceField", "Looking for sample points on the surface...");

		randomSeed_ = 1;
		srand(randomSeed_);
		//srand(time(NULL));

		samplingDensity_ = 25;
		maxTriangleLength_ = 1.0f;
		guidanceFieldEvaluationrange_ = 1.0f;
		Vector3f projectionPosition;

		layout_ = MemoryLayout3D(sfp->getVolumeWidth(), sfp->getVolumeHeight(), sfp->getVolumeDepth());

		// Sample cells through which the surface passes
		GuidanceFieldSample sample = {
					Vector3f(0,0,0),
					0.0f, 
					0.0f,
					0.0f,
					Vector4f(1.0, 0.0f, 0.0f, 1.0f)
				};

		// Set sampling Accuracy
		surface_->SetApproximationAccuracy(0.02f);
		surface_->SetApproximationSteps(20);

		std::vector<std::pair<Vector3f, Spline*>> surfaceSplines = surface_->GetSurfaceSplines();
		int totalNumberOfCells = surfaceSplines.size();
		int current = 0;
		for (auto it = surfaceSplines.begin(); it !=  surfaceSplines.end(); it++){
			int i = samplingDensity_;
			current++;

			// Give debug feedback
			if (current % (totalNumberOfCells/10) == 0){
				char buffer[200];
				sprintf(buffer, "Calculating samples... (%i %% done)", (int)(100.0f * (current/(float)totalNumberOfCells)));
				Log::Info("GuidanceField", buffer);
			}


			while (i > 0) {
				i--;
				
				// Find a position to project from onto the surface
				float x = it->first.X() + (float)rand() / RAND_MAX;
				float y = it->first.Y() + (float)rand() / RAND_MAX;
				float z = it->first.Z() + (float)rand() / RAND_MAX;
				
				// Raycast in each Direction once
				for (int direction = 0; direction <= Surface::Direction::Z; ++direction){	
					// Project
					float projectedPosition = surface_->RaycastInDirection(x, y, z, (Surface::Direction)direction);

					if(projectedPosition != std::numeric_limits<float>::infinity()){
						switch(direction){
							case 0: sample.position = Vector3f(projectedPosition, y, z); break;
							case 1: sample.position = Vector3f(x, projectedPosition, z); break;
							case 2: sample.position = Vector3f(x, y, projectedPosition); break;
						}

						sample.isovalue = surface_->Evaluate(sample.position);

						samples_.push_back(sample);
					}
				}
				

				// Additionally use another projection operator
				Vector3f pos = Vector3f(x, y, z);
				bool success = surface_->ProjectOnSurface(pos, surface_->GetNormal(pos), projectionPosition);

				if (success){
					sample.position = projectionPosition;
					sample.isovalue = surface_->Evaluate(sample.position);
					samples_.push_back(sample);
				}
			}
		}

		char buffer[100];
		sprintf(buffer, "Search completed. Found %i samples.", samples_.size());
		Log::Info("GuidanceField", buffer);
		
		// Now calculate the tensor at each sample position and calculate the ideal edge length with it
		Log::Info("GuidanceField", "Calculating surface tensors for the samples...");

		for (auto it = samples_.begin(); it != samples_.end(); it++){
			// Calculate g
			Vector3f g = Vector3f(
				surface_->EvaluateFirstOrderXDerivative(it->position),
				surface_->EvaluateFirstOrderYDerivative(it->position),
				surface_->EvaluateFirstOrderZDerivative(it->position)
				);

			if(Length(g) <= 0){
				it->debugColor = Vector4f(1.0f, 0.0f, 0.0f, 1.0f);
				continue;
			}

			Vector3f n = g;
			n.Normalize();

			Matrix3f nnT (	n.X() * n.X(),	n.X() * n.Y(),	n.X() * n.Z(),
							n.Y() * n.X(),	n.Y() * n.Y(),	n.Y() * n.Z(),
							n.Z() * n.X(),	n.Z() * n.Y(),	n.Z() * n.Z());

			// Calculate P
			Matrix3f P = Matrix3f::CreateIdentity() - nnT;

			/* Calculate H the Hessian Matrix (	param_type a11, param_type a12, param_type a13, 
												param_type a21, param_type a22, param_type a23, 
												param_type a31, param_type a32, param_type a33)
			*/
			float a21Anda12 = surface_->EvaluateFirstXThenYDerivative(it->position);
			float a31Anda13 = surface_->EvaluateFirstXThenZDerivative(it->position);
			float a32Anda23 = surface_->EvaluateFirstYThenZDerivative(it->position);

			Matrix3f H = Matrix3f(	surface_->EvaluateSecondOrderXDerivative(it->position),	a21Anda12,												a31Anda13,
									a21Anda12,												surface_->EvaluateSecondOrderYDerivative(it->position),	a32Anda23,
									a31Anda13,												a32Anda23,												surface_->EvaluateSecondOrderZDerivative(it->position));

			// Calculate the final matrix G
			Matrix3f G = (P * H * P) / Length(g);

			// Calculate the Trace of G
			const float Trace = Math::Trace(G);
			const auto Frobenius = NormFrobenius(G);

			// Calculate k1 and k2 using qudratic equations
			float sqrt = Math::Sqrt(2*Math::Square(Frobenius) - Math::Square(Trace));

			float k1 = (Trace + sqrt) / 2;
			float k2 = (Trace - sqrt) / 2;

			// Calculate the final result, Kmax. Kmax defines the length of the ideal egde at a point s.
			float Kmax = Max(abs(k1), abs(k2));

			// Set curvature
			it->curvature = Kmax;

			// Calculate the ideal egde length at s
			float edgeLength;

			if(Kmax != 0) {
				//edgeLength = abs((2 * Math::Sin(p_/2)) / Kmax);
				edgeLength = (2 * Math::Sin(p_/2)) / Kmax;
			} else {
				edgeLength = std::numeric_limits<float>::infinity();
			}

			int NumberOfDebugColorLevels = 10;

			if(edgeLength < maxTriangleLength_){
				for (int counter = 1; counter <= NumberOfDebugColorLevels; counter++){

					if(edgeLength < counter * maxTriangleLength_ / NumberOfDebugColorLevels){
						it->debugColor = Vector4f(1.0f - ((float)counter/(float)NumberOfDebugColorLevels), ((float)counter/(float)NumberOfDebugColorLevels), 0.0f, 1.0f);
						break;
					}
				}
				it->idealEdgeLength = edgeLength;

			} else {
				it->debugColor = Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
				it->idealEdgeLength = maxTriangleLength_;
			}
		}

		Log::Info("GuidanceField", "Completed surface tensor calculation!");

		Log::Info("GuidanceField", "Calculating octree for the samples...");

		// Create an octree to be able to traverse the samples efficiently
		std::vector<niven::Vector3f> boundsOffsetTable(8);

		boundsOffsetTable[0] = Vector3f(-0.5, -0.5, -0.5);
		boundsOffsetTable[1] = Vector3f(+0.5, -0.5, -0.5);
		boundsOffsetTable[2] = Vector3f(-0.5, +0.5, -0.5);
		boundsOffsetTable[3] = Vector3f(+0.5, +0.5, -0.5);
		boundsOffsetTable[4] = Vector3f(-0.5, -0.5, +0.5);
		boundsOffsetTable[5] = Vector3f(+0.5, -0.5, +0.5);
		boundsOffsetTable[6] = Vector3f(-0.5, +0.5, +0.5);
		boundsOffsetTable[7] = Vector3f(+0.5, +0.5, +0.5);

		OctreeBoundingBox b;
		b.center = Vector3f(sfp_->getVolumeWidth(), sfp_->getVolumeDepth(), sfp_->getVolumeHeight()) / 2.0f;
		b.radius = sfp_->getVolumeWidth()/2.0f;
		octree_.Build(samples_, boundsOffsetTable, samples_.size(),b);

		// Init octree traversal functions
		getSamplesByDistance_ = [](const Octree &o, const niven::Vector3f pos, float distance, std::vector<GuidanceFieldSample>& data) -> bool 
								{ 
									if(o.PositionIsWithinDistance(pos, distance)){
										if(o.IsLeaf()){
											data.push_back(o.GetLeafSample());
										}
										return true;
									} else {
										return false;
									}
								};
		Log::Info("GuidanceField", "Completed octree calculation!");
	}

float GuidanceField::GetDensity(float x, float y, float z) const {
	return surface_->Evaluate(x, y, z);
}

std::vector<GuidanceFieldSample> GuidanceField::GetSamples() const {
	return samples_;
}

float GuidanceField::G(const GuidanceFieldSample s, const Vector3f x){
	return ((1 - (1 / n_)) * Length(x - s.position)) + (1 / n_) * s.idealEdgeLength;
}


// Schreiner 4.18
float GuidanceField::Evaluate(const Vector3f pos){
	float y = std::numeric_limits<float>::infinity();
	int current = 0;
	std::vector<GuidanceFieldSample> samples;

	octree_.Traverse(getSamplesByDistance_, pos, guidanceFieldEvaluationrange_, samples);

	// Check if no samples were found
	if(samples.empty())
	{
		return maxTriangleLength_;
	}

	// Sort found samples by distance
	std::sort(samples.begin(), samples.end(), [&pos] (const GuidanceFieldSample s1, const GuidanceFieldSample s2) -> bool {
		return LengthSquared(s1.position - pos) < LengthSquared(s2.position - pos);
	});

	do {
		y = Min(y, G(samples[current], pos));
		current++;
	} while (current < samples.size() &&
		Length(samples[current-1].position - pos) <= (y / (1 - (1 / n_))));

	return y;
}

/*
float GuidanceField::Evaluate(const Vector3f pos){
	std::vector<GuidanceFieldSample> samples;

	octree_.Traverse(getSamplesByDistance_, pos, guidanceFieldEvaluationrange_, samples);

	// Check if no samples were found
	if(samples.empty())
	{
		return maxTriangleLength_;
	}

	// Else find he sample with the lowest gs(x)
	GuidanceFieldSample sample =  *(std::min_element(samples.begin(), samples.begin() + samples.size(), [&pos, this] (const GuidanceFieldSample s1, const GuidanceFieldSample s2) -> bool {
		// Evaluate Gs(x) for all the found samples, return the minimum
		// return s1.idealEdgeLength < s2.idealEdgeLength;
		float GsX1 = G(s1, pos);
		float GsX2 = G(s2, pos);
		return GsX1 < GsX2;
	}));

	return G(sample, pos);
}
*/
