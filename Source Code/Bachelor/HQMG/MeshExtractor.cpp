#include "MeshExtractor.h"



MeshExtractor::MeshExtractor(const ShardFileParser::Ptr& sfp, float isoValue, float p, float n)
	: sfp_(sfp),
	isoValue_(isoValue){

		surface_  = SplineSurface::Ptr (new SplineSurface(sfp, isoValue));
		guidanceField_ = GuidanceField::Ptr ( new GuidanceField(sfp, surface_, p, n));
		indiceCounter_ = 0;
		debugMaxTriangleNumber_ = 75;
		debugTriangleCounter_ = debugMaxTriangleNumber_;
		triangleSize_ = 0.5f;
		totalSteps_ = 0;
	}

void MeshExtractor::Setup(){	
	// TODO:: Find more/better seeds
	Vector3f seed = guidanceField_->GetSamples()[0].position;

	// Find a starting front
	GetFrontFromSeed(seed, frontManager_);
}


void MeshExtractor::TriangulateShardFile(std::vector<Vector3f>& vertices, std::vector<Vector3f>& normals, std::vector<Vector4f>& colors, std::vector<uint32>& indices, int steps){

	totalSteps_ += steps;

	char buffer[200];
	sprintf(buffer, "Executing %i steps. Total number of steps is %i", steps, totalSteps_);
	Log::Debug("MeshExtractor", buffer);

	// Vector for finished triangles
	std::vector<Triangle> triangles;

	// Start the algorithm
	while (frontManager_.Count() > 0 && (steps > 0)){
		steps--;
		Front& currentFront = frontManager_.GetRandomFront();
		int currentFrontID = currentFront.ID;
		Front::FrontListIterator currentFrontElementIterator = currentFront.Random();

		// If this front has only three elements it is a finished triangle
		if(currentFront.Size() == 3 && !currentFront.SeedFront){

			triangles.push_back(CreateTriangle(currentFront));

			frontManager_.RemoveFront(currentFrontID);

			if(GiveDebugFeedback(debugTriangleCounter_, debugMaxTriangleNumber_)){
				break;	
			}

			continue;
		}	

		// Attempt to grow a triangle
		Vertex& v1 = currentFrontElementIterator->vertex;
		Vertex& v2 = currentFront.PreviousElement(currentFrontElementIterator)->vertex;
		Vertex attemptVertex;

		// Debug information
		PreOrigin = v1.Position;
		PreNeighbor = v2.Position;
		PreAttempt = v1.Position;
		PostOrigin = v1.Position;
		PostNeighbor = v2.Position;
		PostAttempt = v1.Position;

		if (!GrowVertex(v1, v2, attemptVertex)){
			Log::Debug("MeshExtractor", "Vertex creation failed!");
			continue;
		}

		if (!ProjectVertexOnSurface(attemptVertex)){
			Log::Debug("MeshExtractor", "Projection failed!");
			continue;
		}

		// Update debug information
		PreAttempt = attemptVertex.Position;
		PostAttempt = attemptVertex.Position;

		// Declare for identification of interfering fronts (by reference)
		Front::FrontListIterator intersectingFrontElementIterator;

		if(!CheckFrontInterference(currentFront, attemptVertex, currentFrontElementIterator, intersectingFrontElementIterator))
		{
			// Create the new triangle
			triangles.push_back(CreateTriangle(	v1, 
												v2,
												attemptVertex));
			currentFront.InsertVertex(attemptVertex, currentFrontElementIterator, &frontManager_);

			// Set the seed front back to a a normal front after it has grown 4 vertices
			if(currentFront.Size() > 3 && currentFront.SeedFront){
				currentFront.SeedFront = false;
			}

			if(GiveDebugFeedback(debugTriangleCounter_, debugMaxTriangleNumber_)){
				break;	
			}
		} 
		else 
		{
			if(currentFrontID == intersectingFrontElementIterator->front){
				// Split fronts ( -> Creates a new front)
				currentFront.Split(currentFrontElementIterator, intersectingFrontElementIterator, frontManager_);
			} else {
				// Merge fronts ( -> Removes a front)
				currentFront.Merge(currentFrontElementIterator, intersectingFrontElementIterator, frontManager_);
			}

			// Debug information
			/*
			PostOrigin = currentFrontElementIterator->vertex.Position;
			PostNeighbor = currentFront.PreviousElement(;
			PostAttempt = intersectingFrontElementIterator->vertex.Position;
			*/
		}
	}

	char buffer2[200];
	sprintf(buffer2, "Steps completed. Generated %i triangles.", triangles.size());
	Log::Debug("MeshExtractor", buffer2);

	ParseFrontManager();

	// Finally evaluate the generated triangles
	for (auto it = triangles.begin(); it != triangles.end(); it++){
		vertices.push_back(it->v1.Position);
		normals.push_back(it->v1.Normal);
		colors.push_back(it->v1.Color);
		indices.push_back(indiceCounter_++);

		vertices.push_back(it->v2.Position);
		normals.push_back(it->v2.Normal);
		colors.push_back(it->v1.Color);
		indices.push_back(indiceCounter_++);

		vertices.push_back(it->v3.Position);
		normals.push_back(it->v3.Normal);
		colors.push_back(it->v1.Color);
		indices.push_back(indiceCounter_++);	
	}
}

void MeshExtractor::GetFrontFromSeed(Vector3f seed, FrontManager& fm)
{
	GuidanceFieldSample sample = guidanceField_->Evaluate(seed);

	Vector3f n = surface_->GetNormal(seed);
	Vector3f n2 = Vector3f(n.Y(), n.X(), n.Z());
	Vector3f surfaceVector = Cross(n, n2);

	int newfrontID = fm.CreateFront();

	Vertex v1 = Vertex(seed + 0.5 * sample.idealEdgeLength * surfaceVector);
	Vertex v2 = Vertex(seed - 0.5 * sample.idealEdgeLength * surfaceVector);

	v1.Normal = surface_->GetNormal(v1.Position);
	v2.Normal = surface_->GetNormal(v2.Position);

	fm[newfrontID].AddVertex(v1, &fm);
	fm[newfrontID].AddVertex(v2, &fm);
	fm[newfrontID].SeedFront = true;
}

// Grow vertices per definition to the left using previous element
bool MeshExtractor::GrowVertex(Vertex& v1, Vertex& v2, Vertex& vNew)
{
	// TODO
	if (v1.Position == v2.Position){
		return false;
	}

	// Check if we calculated our vertices already

	if(v1.IdealEdgeLength == std::numeric_limits<float>::infinity()){
		GuidanceFieldSample sample1 = guidanceField_->Evaluate(v1.Position);
		v1.IdealEdgeLength = sample1.idealEdgeLength;
		v1.Color = sample1.debugColor;
	}

	if(v2.IdealEdgeLength == std::numeric_limits<float>::infinity()){
		GuidanceFieldSample sample2 = guidanceField_->Evaluate(v2.Position);
		v2.IdealEdgeLength = sample2.idealEdgeLength;
		v2.Color = sample2.debugColor;
	}

	// Calculate vNew's position

	Vector3f edgeDir = v2.Position - v1.Position;
	const float distance = Length(edgeDir);

	if(distance > (v1.IdealEdgeLength + v2.IdealEdgeLength) || distance < abs(v1.IdealEdgeLength - v2.IdealEdgeLength)){
		// No solution: Circles don't intersect
		return false;
	}

	float a = ((v1.IdealEdgeLength * v1.IdealEdgeLength) - (v2.IdealEdgeLength * v2.IdealEdgeLength) + (distance * distance)) / 2 * distance;
	a = Math::Clamp<float>(a, 0, distance);
	float b = distance - a;

	// Calculate the height. First check if the sqrt would be negative
	if (((v1.IdealEdgeLength * v1.IdealEdgeLength) - (a * a)) < 0){
		// No solution
		return false;
	}

	const float height = sqrt((v1.IdealEdgeLength * v1.IdealEdgeLength) - (a * a));

	// We have the distance of the new point from the edge, now we need to find the direction
	Vector3f edgeNormal = v1.Normal + v2.Normal;
	edgeNormal.Normalize();

	// Now the edge's direction
	edgeDir.Normalize();

	// Calculate the vertex direction
	Vector3f vertexDir = Cross(edgeNormal, edgeDir);

	// Set the position
	vNew.Position = v1.Position + (edgeDir * a) + (vertexDir * height);
	//vNew.Normal = edgeNormal;

	//NIV_DEBUGBREAK_IF_TRUE(vNew.Position.X() != vNew.Position.X());

	return true;
}

bool MeshExtractor::ProjectVertexOnSurface(Vertex& v){
	// TODO Projection in direction of the normal of v! For now attempt all directions and pick the closest result. :/
	std::vector<Vector3f> projections;

	// Raycast in each Direction once
	for (int direction = 0; direction <= Surface::Direction::Z; ++direction){	
		// Project
		float projectedPosition = surface_->RaycastInDirection(v.Position.X(), v.Position.Y(), v.Position.Z(), (Surface::Direction)direction);

		if(projectedPosition != std::numeric_limits<float>::infinity()){
			switch(direction){
				case 0:{
					projections.push_back(Vector3f(projectedPosition, v.Position.Y(), v.Position.Z()));
					break;
					   }
				case 1:{
					projections.push_back(Vector3f(v.Position.X(), projectedPosition, v.Position.Z()));
					break;
					   }
				case 2:{
					projections.push_back(Vector3f(v.Position.X(), v.Position.Y(), projectedPosition));
					break;
					   }
			}

		}
	}

	if(projections.empty()){
		return false;
	}

	v.Position = (*std::min_element(projections.begin(), projections.begin() + projections.size(), [&v] (const Vector3f pos1, const Vector3f pos2) -> bool {
		return Length(pos1-v.Position) < Length(pos2-v.Position);
	}));

	v.Normal = surface_->GetNormal(v.Position);

	// Update Guidance field information for the new vertex
	GuidanceFieldSample sample = guidanceField_->Evaluate(v.Position);
	v.IdealEdgeLength = sample.idealEdgeLength;
	v.Color = sample.debugColor;

	return true;
}

bool MeshExtractor::CheckFrontInterference(Front& current, Vertex newVertex, Front::FrontListIterator& currentFrontElementIterator, Front::FrontListIterator& interferingFrontElementIterator){

	Vector3f originOffset = newVertex.Position;

	// We will test all edges in range against the edges between v1, v2 and newVertex
	Edge edge1;
	edge1.front = current.ID;
	edge1.v1 = newVertex.Position;
	edge1.frontIterator2 = currentFrontElementIterator;
	edge1.v2 = edge1.frontIterator2->vertex.Position;

	Edge edge2;
	edge2.front = current.ID;
	edge2.v1 = newVertex.Position;
	edge2.frontIterator2 = current.PreviousElement(currentFrontElementIterator);
	edge2.v2 = edge2.frontIterator2->vertex.Position;

	// The edges that could have an impact on us, are all but the one between curent and Previous(current). edge1 and edge2 are local variables right now, so they can't be found.
	Edge edgeCurrentPrevious(currentFrontElementIterator, current.PreviousElement(currentFrontElementIterator));
	std::vector<MeshExtractor::Edge> edges = GetEdgesInRange(newVertex, edgeCurrentPrevious);

	// No interference if no elemnts are in range
	if (edges.size() == 0){
		return false;
	}

	// Now transform their coordinate system, so that we can project everything into 2D and intersect
	// Create a coodinate system that is spanned by the vector pointing from the new vertex to the current vertex and a vector orthogonal to that.
	Vector3f u = edge1.v2 - edge1.v1;
	u.Normalize();

	Vector3f w = edge2.v2 - edge2.v1;
	w.Normalize();

	// Find the normal of the new triangle
	Vector3f n = Cross (u, w); 
	n.Normalize();

	Vector3f v = Cross(u, n);
	v.Normalize();

	// Transformation matrix
	Matrix3f coordinateSystem = Matrix3f(	u.X(), u.Y(), u.Z(),
											v.X(), v.Y(), v.Z(),
											n.X(), n.Y(), n.Z());

	// Translate and transform
	TransformEdge(edge1, -originOffset, coordinateSystem);
	TransformEdge(edge2, -originOffset, coordinateSystem);
	TransformEdge(edgeCurrentPrevious, -originOffset, coordinateSystem);

	// For debugging
	LastEdgesInRange.clear();
	LastIntersectionEdges.clear();
	Intersections.clear();

	std::vector<MeshExtractor::Intersection> potentialIntersections;
	MeshExtractor::Intersection currentIntersection;

	// Other elements
	for (auto it = edges.begin(); it != edges.end(); it++){
		TransformEdge((*it), -originOffset, coordinateSystem);
		LastEdgesInRange.push_back((*it));
			
		if ((*it).v2.X() != (*it).v2.X() || (*it).v2.Y() != (*it).v2.Y()){
			int fail = 0;
		}

		// From now on we ignore the z component
		// Test each edge against egde1 and edge2

		if (GetIntersection(edge1, *it, currentIntersection)){
			potentialIntersections.push_back(currentIntersection);
			LastIntersectionEdges.push_back(currentIntersection.edge);
			Intersections.push_back(Vector3f(currentIntersection.x, currentIntersection.y, 0)); 
		}

		if (GetIntersection(edge2, *it, currentIntersection)){
			potentialIntersections.push_back(currentIntersection);
			LastIntersectionEdges.push_back(currentIntersection.edge);
			Intersections.push_back(Vector3f(currentIntersection.x, currentIntersection.y, 0)); 
		}
	}

	// Return if no intersections were found
	if(potentialIntersections.size() == 0){
		return false;
	}

	// Now we only have to find the intersections that fits best. Its still possible to discard all intersections, if they are just
	// intersections with our original edge
	if (!FindBestIntersection(potentialIntersections, edgeCurrentPrevious, interferingFrontElementIterator, currentFrontElementIterator)){
		return false;
	}

	return true;
}

std::vector<MeshExtractor::Edge> MeshExtractor::GetEdgesInRange(const Vertex& v, const MeshExtractor::Edge& edgeCurrentPrevious){
	std::vector<MeshExtractor::Edge> elementsInRange;

	// Ideal edge length has ben calculated before. Factor 2 because vertices could be outside the ideal length sphere and still intersect
	float range = v.IdealEdgeLength * 2;

	// TODO: Octree or any other spatial subdivision for faster traversal!
	for (auto frontIDIt = frontManager_.FrontsBegin(); frontIDIt != frontManager_.FrontsEnd(); frontIDIt++){
		Front& f = frontIDIt->second;

		for (auto frontElementIt = f.Elements.begin(); frontElementIt != f.Elements.end(); frontElementIt++){
			if (Length(v.Position - frontElementIt->vertex.Position) < range && 
				frontElementIt->ID != edgeCurrentPrevious.frontIterator1->ID &&
				f.PreviousElement(frontElementIt)->ID != edgeCurrentPrevious.frontIterator2->ID){
				// Make a copy and push back
				MeshExtractor::Edge e(frontElementIt, f.PreviousElement(frontElementIt));
				elementsInRange.push_back(e);
			}
		}
	}

	return elementsInRange;
}

bool MeshExtractor::GetIntersection(const MeshExtractor::Edge& e1, const MeshExtractor::Edge& e2, MeshExtractor::Intersection& intersection){
	float x1 = e1.v1.X(), x2 = e1.v2.X(), x3 = e2.v1.X(), x4 = e2.v2.X();
	float y1 = e1.v1.Y(), y2 = e1.v2.Y(), y3 = e2.v1.Y(), y4 = e2.v2.Y();

	 
	float denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
	// If the denominator is zero, the lines must be parallel
	if (denominator == 0) {
		return false;
	}
 
	// Get the x and y coordinates of the intersection
	float s = (x1 * y2 - y1 * x2), t = (x3 * y4 - y3 * x4);
	float x = (s * (x3 - x4) - (x1 - x2) * t) / denominator;
	float y = (s * (y3 - y4) - (y1 - y2) * t) / denominator;
 
	// Check if the x and y coordinates are within both lines
	if (	x < Min(x1, x2) ||
			x > Max(x1, x2) ||
			x < Min(x3, x4) ||
			x > Max(x3, x4) ||
			y < Min(y1, y2) || 
			y > Max(y1, y2) ||
			y < Min(y3, y4) || 
			y > Max(y3, y4)) {
		return false;
	}

	intersection.x = x;
	intersection.y = y;
	intersection.edge = e2;
	intersection.origin = e1.frontIterator2;

	return true;
}

bool MeshExtractor::FindBestIntersection(const std::vector<MeshExtractor::Intersection>& intersections, const MeshExtractor::Edge& originalEdge, Front::FrontListIterator& bestIntersection, Front::FrontListIterator& bestOrigin){

	// We now have a list of edges interfering with us.
	// v1 := originalEdge.v1 v2:= originalEdge.v2
	// The final intersecion should fullfill the following criteria:
	// 1) It shouldn't be an intersection with v1 or v2
	// 2) Its distance to v1 and v2 should be minimal.
	// 3) The edges between the intersection vertex and v1 and v2 should not lead to new intersections.
	// 3.1) This should be guaranteed by 2) maybe /TODO

	const Vector2f v1(originalEdge.v1.X(), originalEdge.v1.Y());
	const Vector2f v2(originalEdge.v2.X(), originalEdge.v2.Y());
	bool foundIntersection = false;

	float currentMinimalDistance = std::numeric_limits<float>::infinity();

	for (auto it = intersections.begin(); it != intersections.end(); it++){
			
		// Current intersection
		const Vector2f intersection(it->x, it->y);
		const Vector2f intersectionv1(it->edge.v1.X(), it->edge.v1.Y());
		const Vector2f intersectionv2(it->edge.v2.X(), it->edge.v2.Y());

		// First test 1)
		if (intersection == v1 || intersection == v2){
			continue;
		}

		// Note that now we already MUST (and will) choose an intersection
		foundIntersection = true;

		// Note that originalEdge has been transformed into our 2D world.
		float accumulatedDistance = Length(intersection - v1) + Length(intersection - v2);

		if (accumulatedDistance < currentMinimalDistance){
			// This intersection is a minimal candidate. 
			currentMinimalDistance = accumulatedDistance;
			// Find the vertex of the edge that is not v1 or v2 and closest to the intersection
			if (Length(intersectionv1 - intersection) < Length(intersectionv1 - intersection)){
				if (it->edge.frontIterator1->vertex.Position != originalEdge.frontIterator1->vertex.Position && it->edge.frontIterator1->vertex.Position != originalEdge.frontIterator2->vertex.Position){
					bestIntersection = it->edge.frontIterator1;
				} else {
					bestIntersection = it->edge.frontIterator2;
				} 
			} else {
				if (it->edge.frontIterator2->vertex.Position != originalEdge.frontIterator1->vertex.Position && it->edge.frontIterator2->vertex.Position != originalEdge.frontIterator2->vertex.Position){
					bestIntersection = it->edge.frontIterator2;
				} else {
					bestIntersection = it->edge.frontIterator1;
				} 
			}

			bestOrigin = it->origin;

			// Only reason not to use this origin would be, if origin and intersection were direct neighbors
			if (bestOrigin->front == bestIntersection->front){
				Front& f = frontManager_[bestOrigin->front];

				if (f.PreviousElement(bestOrigin)->ID == bestIntersection->ID){
					bestOrigin = f.NextElement(bestOrigin);
				}

				if (f.NextElement(bestOrigin)->ID == bestIntersection->ID){
					bestOrigin = f.PreviousElement(bestOrigin);
				}
			}
		}
	
		PostOrigin = bestOrigin->vertex.Position;
		PostAttempt = bestIntersection->vertex.Position;
	}

	return foundIntersection;
}

void MeshExtractor::TransformEdge(MeshExtractor::Edge& edge, const Vector3f& translation, const Matrix3f& transformation){
	edge.v1 += translation;
	edge.v2 += translation;
	edge.v1 = transformation * edge.v1;
	edge.v2 = transformation * edge.v2;
}


MeshExtractor::Triangle MeshExtractor::CreateTriangle(Front& f){

	Vertex v1 = f.Elements.front().vertex;
	f.Elements.pop_front();

	Vertex v2 = f.Elements.front().vertex;
	f.Elements.pop_front();

	Vertex v3 = f.Elements.front().vertex;

	return CreateTriangle(v1, v2, v3);
}

MeshExtractor::Triangle MeshExtractor::CreateTriangle(Vertex& v1, Vertex& v2, Vertex& v3){	
	Triangle t;
	t.v1 = v1;
	t.v2 = v2;
	t.v3 = v3;
	return t;
}

bool MeshExtractor::GiveDebugFeedback(int& debugTriangleCounter, int max){
		debugTriangleCounter--;

		if(debugTriangleCounter % (max / 10) == 0){
			char buffer[200];
			sprintf(buffer, "Triangulating surface... (%i %%)", (int)(((max - debugTriangleCounter) / (float)max) * 100.0f));
			Log::Debug("MeshExtractor", buffer);
		}

		if(debugTriangleCounter <= 0){
			return true;
		} else {
			return false;
		}
}

void MeshExtractor::ParseFrontManager(){
	DebugFront.clear();

	for (auto frontIDIt = frontManager_.FrontsBegin(); frontIDIt != frontManager_.FrontsEnd(); frontIDIt++){
		Front& f = frontIDIt->second;
		std::vector<Vector3f> front;

		for (auto frontElementIt = f.Elements.begin(); frontElementIt != f.Elements.end(); frontElementIt++){
			front.push_back(frontElementIt->vertex.Position);
		}

		DebugFront.push_back(front);
	}
}