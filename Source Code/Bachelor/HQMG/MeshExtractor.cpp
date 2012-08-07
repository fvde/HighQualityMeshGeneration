#include "MeshExtractor.h"



MeshExtractor::MeshExtractor(const ShardFileParser::Ptr& sfp, float isoValue, float p, float n)
	: sfp_(sfp),
	isoValue_(isoValue){

		surface_  = SplineSurface::Ptr (new SplineSurface(sfp, isoValue));
		guidanceField_ = GuidanceField::Ptr ( new GuidanceField(sfp, surface_, p, n));
	}



void MeshExtractor::TriangulateShardFile(std::vector<Vector3f>& vertices, std::vector<Vector3f>& normals, std::vector<Vector4f>& colors, std::vector<uint32>& indices){

	Log::Debug("MeshExtractor", "Starting the advancing front algorithm...");

	float triangleSize = 0.5f;
	int indiceCounter = indices.size();
	int debugMaxTriangleNumber = 100;
	int debugTriangleCounter = debugMaxTriangleNumber;

	// TODO:: Find more/better seeds
	Vector3f seed = guidanceField_->GetSamples()[0].position;

	// Find a starting front
	GetFrontFromSeed(seed, frontManager_);

	// Vector for finished triangles
	std::vector<Triangle> triangles;

	// Start the algorithm
	while (frontManager_.Count() > 0){

		Front& currentFront = frontManager_.GetRandomFront();
		int currentFrontID = currentFront.ID;
		Front::FrontListIterator currentFrontElementIterator = currentFront.Random();

		// If this front has only three elements it is a finished triangle
		if(currentFront.Size() == 3 && !currentFront.SeedFront){
			// TODO Check if its a seed triangle

			Vertex v1 = currentFront.Elements.front().vertex;
			currentFront.Elements.pop_front();

			Vertex v2 = currentFront.Elements.front().vertex;
			currentFront.Elements.pop_front();

			Vertex v3 = currentFront.Elements.front().vertex;

			triangles.push_back(CreateTriangle(v1, v2, v3));

			frontManager_.RemoveFront(currentFrontID);
			continue;
		}	

		// Attempt to grow a triangle
		//Vertex attemptVertex = GrowVertex(currentFrontElementIterator->vertex, currentFrontElementIterator->front->PreviousElement(currentFrontElementIterator)->vertex);
		Vertex& v1 = currentFrontElementIterator->vertex;
		Vertex& v2 = currentFront.PreviousElement(currentFrontElementIterator)->vertex;
		Vertex attemptVertex;

		if (!GrowVertex(v1, v2, attemptVertex)){
			Log::Debug("MeshExtractor", "Vertex creation failed!");
			continue;
		}

		if (!ProjectVertexOnSurface(attemptVertex)){
			Log::Debug("MeshExtractor", "Projection failed!");
			continue;
		}

		// Declare for identification of interfering fronts (by reference)
		Front::FrontListIterator intersectingFrontElementIterator;
		bool selfIntersection = false;

		if(!CheckFrontInterference(currentFront, attemptVertex, currentFrontElementIterator, intersectingFrontElementIterator, selfIntersection))
		{
			// Create the new triangle
			triangles.push_back(CreateTriangle(	v1, 
												v2,
												attemptVertex));
			currentFront.InsertVertex(attemptVertex, currentFrontElementIterator, &frontManager_);

			// Set the seed front back to a anormal front after it has grown 4 vertices
			if(currentFront.Size() > 3 && currentFront.SeedFront){
				currentFront.SeedFront = false;
			}

			if(GiveDebugFeedback(debugTriangleCounter, debugMaxTriangleNumber)){
				break;	
			}
		} 
		else 
		{
			if(selfIntersection){
				// Don't create a triangle here, since it would only be a line or a point. Skip and hope that this triangle will be created from another position with better shape.
				continue;
			}

			if(currentFrontID == intersectingFrontElementIterator->front){
				// Split fronts ( -> Creates a new front)
				currentFront.Split(currentFrontElementIterator, intersectingFrontElementIterator, frontManager_);
			} else {
				// Merge fronts ( -> Removes a front)
				currentFront.Merge(currentFrontElementIterator, intersectingFrontElementIterator, frontManager_);
			}

			// Merging and splitting also results in a 'connection' triangle which we can add to our triangulation
			triangles.push_back(CreateTriangle(	v1, 
												v2,
												intersectingFrontElementIterator->vertex));
			if(GiveDebugFeedback(debugTriangleCounter, debugMaxTriangleNumber)){
				break;	
			}
		}
	}

	char buffer[200];
	sprintf(buffer, "Advancing front algorithm completed. Generated %i triangles.", triangles.size());
	Log::Debug("MeshExtractor", buffer);

	// Finally evaluate the generated triangles
	for (auto it = triangles.begin(); it != triangles.end(); it++){
		vertices.push_back(it->v1.Position);
		normals.push_back(it->v1.Normal);
		colors.push_back(it->v1.Color);
		indices.push_back(indiceCounter++);

		vertices.push_back(it->v2.Position);
		normals.push_back(it->v2.Normal);
		colors.push_back(it->v1.Color);
		indices.push_back(indiceCounter++);

		vertices.push_back(it->v3.Position);
		normals.push_back(it->v3.Normal);
		colors.push_back(it->v1.Color);
		indices.push_back(indiceCounter++);	
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
		// No solution
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
	vNew.Normal = edgeNormal;

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

bool MeshExtractor::CheckFrontInterference(Front& current, Vertex newVertex, Front::FrontListIterator& currentFrontElementIterator, Front::FrontListIterator& interferingFrontElementIterator, bool& selfIntersection){

	Vector3f originOffset = newVertex.Position;

	// We will test all edges in range against the edges between v1, v2 and newVertex
	Edge edge1;
	edge1.front = current.ID;
	edge1.v1 = newVertex.Position;
	edge1.v2 = currentFrontElementIterator->vertex.Position;

	Edge edge2;
	edge2.front = current.ID;
	edge2.v1 = newVertex.Position;
	edge2.v2 = current.PreviousElement(currentFrontElementIterator)->vertex.Position;

	std::vector<MeshExtractor::Edge> edges = GetEdgesInRange(newVertex, edge1.v2, edge2.v2);

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

	std::vector<MeshExtractor::Intersection> potentialIntersections;
	MeshExtractor::Intersection currentIntersection;

	// Other elements
	for (auto it = edges.begin(); it != edges.end(); it++){
		TransformEdge((*it), -originOffset, coordinateSystem);

		// From now on we ignore the z component
		// Test each edge against egde1 and edge2

		if (GetIntersection(edge1, *it, currentIntersection)){
			potentialIntersections.push_back(currentIntersection);
		}

		if (GetIntersection(edge2, *it, currentIntersection)){
			potentialIntersections.push_back(currentIntersection);
		}
	}

	// Finally check which of the found intersections is closest to the original edge
	if(potentialIntersections.size() == 0){
		return false;
	} else if (potentialIntersections.size() == 1){
		interferingFrontElementIterator = potentialIntersections[0].frontIterator;
	} else {

		// Initialize
		interferingFrontElementIterator = potentialIntersections[0].frontIterator;
		float currentMinimalDistance = std::numeric_limits<float>::infinity();

		for (auto it = potentialIntersections.begin(); it != potentialIntersections.end(); it++){
			
			const Vector2f i = Vector2f(it->x, it->y);

			// Note that edge1 and edge2 are transformed into out 2D world.
			float accumulatedDistance = Length(i - Vector2f(edge1.v2.X(), edge1.v2.Y())) + Length(i - Vector2f(edge2.v2.X(), edge2.v2.Y()));

			if(accumulatedDistance < currentMinimalDistance && !IsCloseSelfIntersection(it->frontIterator, currentFrontElementIterator)){
				currentMinimalDistance = accumulatedDistance;
				interferingFrontElementIterator = it->frontIterator;
			}
		}
	}

	// Check if we are not intersecting with ourselves or an adjacent front element
	if (IsCloseSelfIntersection(interferingFrontElementIterator, currentFrontElementIterator)){
			// Bad triangle
			selfIntersection = true;
	}

	return true;
}

std::vector<MeshExtractor::Edge> MeshExtractor::GetEdgesInRange(Vertex& v, const Vector3f& v1, const Vector3f& v2){
	std::vector<MeshExtractor::Edge> elementsInRange;

	// Ideal edge length has ben calculated before. Factor 2 because vertices could be outside the ideal length sphere and still intersect
	float range = v.IdealEdgeLength * 2;

	// TODO: Octree or any other spatial subdivision for faster traversal!
	for (auto frontIDIt = frontManager_.FrontsBegin(); frontIDIt != frontManager_.FrontsEnd(); frontIDIt++){
		Front& f = frontIDIt->second;

		for (auto frontElementIt = f.Elements.begin(); frontElementIt != f.Elements.end(); frontElementIt++){
			if (Length(v.Position - frontElementIt->vertex.Position) < range && 
				frontElementIt->vertex.Position != v.Position &&
				frontElementIt->vertex.Position != v1 &&
				frontElementIt->vertex.Position != v2 &&
				f.PreviousElement(frontElementIt)->vertex.Position != v.Position &&
				f.PreviousElement(frontElementIt)->vertex.Position != v1 &&
				f.PreviousElement(frontElementIt)->vertex.Position != v2){
				// Make a copy and push back
				MeshExtractor::Edge e;
				e.front = frontIDIt->first;
				e.frontIterator1 = frontElementIt;
				e.frontIterator2 = f.PreviousElement(frontElementIt);
				e.v1 = e.frontIterator1->vertex.Position;
				e.v2 = e.frontIterator2->vertex.Position;
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
	
	// Find the closer front element // TODO: Might be the other way around?
	if(Length(Vector2f(x, y) - Vector2f(x3, y3)) < Length(Vector2f(x, y) - Vector2f(x4, y4))){
		intersection.frontIterator = e2.frontIterator1;
	} else {
		intersection.frontIterator = e2.frontIterator2;
	}

	return true;
}

bool MeshExtractor::IsCloseSelfIntersection(Front::FrontListIterator& interfering, Front::FrontListIterator& current){
	Front& f1 = frontManager_[current->front];
	Front& f2 = frontManager_[interfering->front];
	
	if (f1.ID != f2.ID){
		return false;
	}

	return (interfering == current ||
		interfering == f1.PreviousElement(current) ||
		interfering == f1.NextElement(current));
}

void MeshExtractor::TransformEdge(MeshExtractor::Edge& edge, const Vector3f& translation, const Matrix3f& transformation){
	edge.v1 += translation;
	edge.v2 += translation;
	edge.v1 = transformation * edge.v1;
	edge.v2 = transformation * edge.v2;
}


MeshExtractor::Triangle MeshExtractor::CreateTriangle(Vertex v1, Vertex v2, Vertex v3){
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