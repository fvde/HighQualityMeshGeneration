#include "GuidanceField.h"



GuidanceField::GuidanceField(const ShardFileParser::Ptr& sfp, Surface::Ptr s, float p, float n)
	:	sfp_(sfp),
		p_(p),
		n_(n),
		surface_(s){

		Log::Debug("GuidanceField", "Looking for sample points on the surface...");

		randomSeed_ = 3;
		srand(randomSeed_);
		//srand(time(NULL));

		samplingDensity_ = 50;
		maxTriangleLength_ = 1.0f;

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
		for (auto it = surfaceSplines.begin(); it !=  surfaceSplines.end(); it++){
			int i = samplingDensity_;
			while (i > 0) {
				i--;
				// Raycast in each Direction once
				for (int direction = 0; direction <= Surface::Direction::Z; ++direction){	

					// Find a position to project from onto the surface
					float x = it->first.X() + (float)rand() / RAND_MAX;
					float y = it->first.Y() + (float)rand() / RAND_MAX;
					float z = it->first.Z() + (float)rand() / RAND_MAX;

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
			}
		}

		char buffer[100];
		sprintf(buffer, "Search completed. Found %i samples.", samples_.size());
		Log::Debug("GuidanceField", buffer);
		
		// Now calculate the tensor at each sample position and calculate the ideal edge length with it
		Log::Debug("GuidanceField", "Calculating surface tensors for the samples...");


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
			float Kmax = 0.0f;

			if(k1 > k2){
				Kmax = k1;
			} else {
				Kmax = k2;
			}

			// Set curvature
			it->curvature = Kmax;

			// Calculate the ideal egde length at s
			float edgeLength;

			if(Kmax != 0) {
				// ATTENTION: -1 Added although its not in the paper. But else everything would be negative.
				edgeLength = (-1) * (2 * Math::Sin(p_/2)) / Kmax;
			} else {
				edgeLength = std::numeric_limits<float>::infinity();
			}

			int NumberOfDebugColorLevels = 10;

			if(edgeLength < maxTriangleLength_){
				for (int counter = 1; counter <= NumberOfDebugColorLevels; counter++){

					if(edgeLength < counter * maxTriangleLength_ / NumberOfDebugColorLevels){
						//it->debugColor = Vector4f(1.0f - ((float)counter/(float)NumberOfDebugColorLevels), 0.0f, ((float)counter/(float)NumberOfDebugColorLevels), 1.0f);
						it->debugColor = Vector4f((float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX, 1);
						break;
					}
				}
				it->idealEdgeLength = edgeLength;
			} else {
				it->debugColor = Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
				it->idealEdgeLength = maxTriangleLength_;
			}
		}

		Log::Debug("GuidanceField", "Completed surface tensor calculation!");

		Log::Debug("GuidanceField", "Calculating octree for the samples...");

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

		sampleAdaptivityRange_ = 0.2f;

		Log::Debug("GuidanceField", "Completed octree calculation!");
	}

float GuidanceField::GetDensity(float x, float y, float z) const {
	return surface_->Evaluate(x, y, z);
}

std::vector<GuidanceFieldSample> GuidanceField::GetSamples() const {
	return samples_;
}


GuidanceFieldSample GuidanceField::Evaluate(Vector3f pos){
	std::vector<GuidanceFieldSample> samples;
	octree_.Traverse(getSamplesByDistance_, pos, sampleAdaptivityRange_, samples);

	// Check if no samples were found
	if(samples.empty())
	{
		return GuidanceFieldSample();
	}
	// Else return the lowest ideal edge length
	return *(std::min_element(samples.begin(), samples.begin() + samples.size(), [] (const GuidanceFieldSample s1, const GuidanceFieldSample s2) {
		return s1.idealEdgeLength < s2.idealEdgeLength;
	}));
}
