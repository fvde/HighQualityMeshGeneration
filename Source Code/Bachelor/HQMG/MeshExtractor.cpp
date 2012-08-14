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

		// If this front has only three elements it is a finished triangle
		if(currentFront.Size() == 3 && !currentFront.SeedFront){
			triangles.push_back(CreateTriangle(currentFront));
			frontManager_.RemoveFront(currentFrontID);
			continue;
		}	

		// Attempt to grow a triangle
		Front::FrontListIterator bestOrigin = currentFront.Random();
		Front::FrontListIterator bestNeighbor = currentFront.PreviousElement(bestOrigin);

		Vertex& v1 = bestOrigin->vertex;
		Vertex& v2 = bestNeighbor->vertex;
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
		Front::FrontListIterator bestIntersection;

		if(!CheckFrontInterference(attemptVertex, bestOrigin, bestNeighbor, bestIntersection))
		{
			// Create the new triangle
			triangles.push_back(CreateTriangle(	v1, 
												v2,
												attemptVertex));
			currentFront.InsertVertex(attemptVertex, bestOrigin, &frontManager_);

			// Set the seed front back to a a normal front after it has grown 4 vertices
			if(currentFront.Size() > 3 && currentFront.SeedFront){
				currentFront.SeedFront = false;
			}
		} 
		else 
		{
			if(currentFrontID == bestIntersection->front){
				// Split fronts ( -> Creates a new front)
				currentFront.Split(bestOrigin, bestIntersection, frontManager_);
			} else {
				// Merge fronts ( -> Removes a front)
				currentFront.Merge(bestOrigin, bestIntersection, frontManager_);
			}
		}
	}

	char buffer2[200];
	sprintf(buffer2, "Steps completed. Generated %i triangles.", triangles.size());
	Log::Debug("MeshExtractor", buffer2);

	// Debugging
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

bool MeshExtractor::CheckFrontInterference(Vertex newVertex, Front::FrontListIterator& bestOrigin, Front::FrontListIterator& bestNeighbor, Front::FrontListIterator& bestIntersection){

	Front& current = frontManager_[bestOrigin->front];
	Vector3f originOffset = newVertex.Position;

	// We will test all edges in range against the edges between v1, v2 and newVertex
	Edge edge1;
	edge1.front = current.ID;
	edge1.v1 = newVertex.Position;
	edge1.frontIterator2 = bestOrigin;
	edge1.v2 = edge1.frontIterator2->vertex.Position;

	Edge edge2;
	edge2.front = current.ID;
	edge2.v1 = newVertex.Position;
	edge2.frontIterator2 = bestNeighbor;
	edge2.v2 = edge2.frontIterator2->vertex.Position;

	// The edges that could have an impact on us, are all but the one between curent and Previous(current). edge1 and edge2 are local variables right now, so they can't be found.
	Edge edgeOriginNeighbor(bestOrigin, bestNeighbor);
	std::vector<MeshExtractor::Edge> edges = GetEdgesInRange(newVertex, edgeOriginNeighbor);

	// No interference if no elements are in range
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
	TransformEdge(edgeOriginNeighbor, -originOffset, coordinateSystem);

	for (auto it = edges.begin(); it != edges.end(); it++){
		TransformEdge((*it), -originOffset, coordinateSystem);
	}

	// For debugging
	LastEdgesInRange.clear();
	LastIntersectionEdges.clear();
	Intersections.clear();

	std::vector<MeshExtractor::Intersection> potentialIntersections;

	TestEdge(edge1, edges, potentialIntersections);
	TestEdge(edge2, edges, potentialIntersections);

	// More Debug Information:
	TransformedPostOrigin = edgeOriginNeighbor.v1;
	TransformedPostNeighbor = edgeOriginNeighbor.v2;
	TransformedPostAttempt = edge1.v1;

	// Return if no intersections were found
	if(potentialIntersections.size() == 0){
		return false;
	}
	
	// For all found intersections get the points within the triangle
	std::vector<MeshExtractor::Intersection> insideTriangleIntersections;
	MeshExtractor::Intersection intersection;
	intersection.origin = edgeOriginNeighbor.frontIterator1;
	// Temporary // TODO
	// Add all points inside the triangle as intersections
	/*
	for (auto it = edges.begin(); it != edges.end(); it++){

		intersection.edge = (*it);
		FindPointWithinTriangle(intersection, Vector2f(edge1.v1.X(), edge1.v1.Y()),
										Vector2f(edge1.v2.X(), edge1.v2.Y()),
										Vector2f(edge2.v2.X(), edge2.v2.Y()),
										insideTriangleIntersections);
	}
	*/
	
	for (auto it = potentialIntersections.begin(); it != potentialIntersections.end(); it++){
		FindPointWithinTriangle((*it), Vector2f(edge1.v1.X(), edge1.v1.Y()),
										Vector2f(edge1.v2.X(), edge1.v2.Y()),
										Vector2f(edge2.v2.X(), edge2.v2.Y()),
										insideTriangleIntersections);
	}
	

	potentialIntersections.insert(potentialIntersections.begin(), insideTriangleIntersections.begin(), insideTriangleIntersections.begin() + insideTriangleIntersections.size());

	// Now we only have to find the intersections that fits best.
	FindBestIntersection(potentialIntersections, edgeOriginNeighbor, bestOrigin, bestNeighbor, bestIntersection);

	// We have found the ideal intersection and origin. But this intersection could lead to new intersections if we're unlucky.
	/*
	int safety = 5;
	do {
		safety--;

		Edge originIntersection (bestIntersection, bestOrigin);
		Edge neighborIntersection (bestIntersection, bestNeighbor);
		Edge newCurentNewPrevious(bestOrigin, bestNeighbor);
		potentialIntersections.clear();
		insideTriangleIntersections.clear();

		// Find edges
		edges = GetEdgesInRange(bestOrigin->vertex, newCurentNewPrevious);

		// Transform edges
		TransformEdge(originIntersection, -originOffset, coordinateSystem);
		TransformEdge(neighborIntersection, -originOffset, coordinateSystem);
		TransformEdge(newCurentNewPrevious, -originOffset, coordinateSystem);
		for (auto it = edges.begin(); it != edges.end(); it++){
			TransformEdge((*it), -originOffset, coordinateSystem);
		}

		// Test
		LastEdgesInRange.clear();
		LastIntersectionEdges.clear();
		TestEdge(originIntersection, edges, potentialIntersections);
		TestEdge(neighborIntersection, edges, potentialIntersections);

		// For all found intersections get the points within the triangle
		for (auto it = potentialIntersections.begin(); it != potentialIntersections.end(); it++){
			FindPointWithinTriangle((*it), Vector2f(originIntersection.v1.X(), originIntersection.v1.Y()),
										Vector2f(originIntersection.v2.X(), originIntersection.v2.Y()),
										Vector2f(neighborIntersection.v2.X(), neighborIntersection.v2.Y()),
										insideTriangleIntersections);
		}

		potentialIntersections.insert(potentialIntersections.begin(), insideTriangleIntersections.begin(), insideTriangleIntersections.begin() + insideTriangleIntersections.size());


		// Debbuging
		for (auto it = potentialIntersections.begin(); it != potentialIntersections.end(); it++){
			AdditionalDebugInfo.push_back(Vector3f(it->x, it->y, 0));
		}

		// We found intersections, now we have to identify the best intersection and origin once again. Greater than 1 because the list will include the intersecion with bestIntersection, which is fine.
		if(potentialIntersections.size() > 2){
			FindBestIntersection(potentialIntersections, newCurentNewPrevious, bestOrigin, bestNeighbor, bestIntersection);
		}
	} while (potentialIntersections.size() > 2 && safety > 0);
	*/

	return true;
}

std::vector<MeshExtractor::Edge> MeshExtractor::GetEdgesInRange(const Vertex& v, const MeshExtractor::Edge& edgeCurrentPrevious){
	std::vector<MeshExtractor::Edge> elementsInRange;

	// Ideal edge length has ben calculated before. Factor 2 because vertices could be outside the ideal length sphere and still intersect
	float range = v.IdealEdgeLength * 3;

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

void MeshExtractor::TestEdge(const MeshExtractor::Edge& edge, const std::vector<MeshExtractor::Edge>& edgesInRange, std::vector<MeshExtractor::Intersection>& intersections){

	// The Edge MUST by transformed into the adequate 2D space beforehand!

	MeshExtractor::Intersection currentIntersection;
	float zCoordinateAcceptance = 0.5f;

	// Other elements
	for (auto it = edgesInRange.begin(); it != edgesInRange.end(); it++){
		const Edge& e = *it;
		LastEdgesInRange.push_back(e);

		// Discard edges which are not intersecting our slice of the mesh
		if(!TestZCoordinate(e, zCoordinateAcceptance)){
			continue;
		}
		
			
		if (e.v2.X() != e.v2.X() || e.v2.Y() != e.v2.Y()){
			int fail = 0;
		}

		// From now on we ignore the z component
		// Test each edge against egde1 and edge2

		if (GetIntersection(edge, e, currentIntersection)){
			intersections.push_back(currentIntersection);
			LastIntersectionEdges.push_back(currentIntersection.edge);
			Intersections.push_back(Vector3f(currentIntersection.x, currentIntersection.y, 0));
		}
	}
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

	Vector2f i(x, y);
	Vector2f v(e1.v2.X(), e1.v2.Y());

	// Test if its an intersection with ourselves
	if (IsSame2DVertex(i, v)){
		return false;
	}

	intersection.x = x;
	intersection.y = y;
	intersection.edge = e2;
	intersection.origin = e1.frontIterator2;
	intersection.insideTriangle = false;

	return true;
}

void MeshExtractor::FindPointWithinTriangle(const MeshExtractor::Intersection& intersection, const Vector2f& v1, const Vector2f& v2, const Vector2f& v3, std::vector<MeshExtractor::Intersection>& intersections){

	if (MeshExtractor::IsInsideTriangle(v1, v2, v3, Vector2f(intersection.edge.v1.X(), intersection.edge.v1.Y()))){
		Intersection i(intersection);
		i.x = intersection.edge.v1.X();
		i.y = intersection.edge.v1.Y();
		i.insideTriangle = true;

		Intersections.push_back(Vector3f(i.x, i.y, 0));
		intersections.push_back(i);
	} 
	else  if (MeshExtractor::IsInsideTriangle(v1, v2, v3, Vector2f(intersection.edge.v2.X(), intersection.edge.v2.Y()))){
		Intersection i(intersection);
		i.x = intersection.edge.v2.X();
		i.y = intersection.edge.v2.Y();
		i.insideTriangle = true;

		Intersections.push_back(Vector3f(i.x, i.y, 0));
		intersections.push_back(i);
	}
}

bool MeshExtractor::IsInsideTriangle(const Vector2f& a, const Vector2f& b, const Vector2f& c, const Vector2f& point){
	Vector2f v0 = c - a;
	Vector2f v1 = b - a;
	Vector2f v2 = point - a;

	// Compute dot products
	float dot00 = Dot(v0, v0);
	float dot01 = Dot(v0, v1);
	float dot02 = Dot(v0, v2);
	float dot11 = Dot(v1, v1);
	float dot12 = Dot(v1, v2);

	// Compute barycentric coordinates
	float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
	float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

	// Check if point is in triangle
	return (u >= 0) && (v >= 0) && (u + v < 1);
}

void MeshExtractor::FindBestIntersection(const std::vector<MeshExtractor::Intersection>& intersections, const MeshExtractor::Edge& originalEdge, Front::FrontListIterator& bestOrigin, Front::FrontListIterator& bestNeighbor, Front::FrontListIterator& bestIntersection){

	// We now have a list of edges interfering with us.
	// v1 := originalEdge.v1 v2:= originalEdge.v2
	// The final intersecion should fullfill the following criteria:
	// 1) It shouldn't be an intersection with v1 or v2: Guaranteed by GetIntersection.
	// 2) Its distance to v1 and v2 should be minimal.
	// 3) The edges between the intersection vertex and v1 and v2 should not lead to new intersections.
	// 3.1) This should be guaranteed by 2) maybe /TODO

	const Vector2f v1(originalEdge.v1.X(), originalEdge.v1.Y());
	const Vector2f v2(originalEdge.v2.X(), originalEdge.v2.Y());

	float currentMinimalDistance = std::numeric_limits<float>::infinity();
	float curentInsideMinimalDistance = currentMinimalDistance;

	for (auto it = intersections.begin(); it != intersections.end(); it++){
			
		// Current intersection
		const Vector2f intersection(it->x, it->y);
		const Vector2f intersectionv1(it->edge.v1.X(), it->edge.v1.Y());
		const Vector2f inte rsectionv2(it->edge.v2.X(), it->edge.v2.Y());

		// Note that originalEdge has been transformed into our 2D world.
		float accumulatedDistance = Length(intersection - v1) + Length(intersection - v2);

		// Decide if its a good intersection to pick
		if (IsBetterIntersection((*it), accumulatedDistance, curentInsideMinimalDistance, currentMinimalDistance, originalEdge.front)) 
		{
			// This intersection is a minimal candidate. 
			currentMinimalDistance = accumulatedDistance;

			if (it->insideTriangle){
				curentInsideMinimalDistance = accumulatedDistance;
			}

			// Find the vertex of the edge that is not v1 or v2 and closest to the intersection
			bestIntersection = FindBestIntersectionElement((*it), originalEdge);
			bestOrigin = it->origin;

			// Find best neighbor. It has to be part of the original edge.
			if (originalEdge.frontIterator1->ID == bestOrigin->ID){
				bestNeighbor = originalEdge.frontIterator2;
			} else {
				bestNeighbor = originalEdge.frontIterator1;
			}

			// Only reason not to use this origin would be, if origin and intersection were direct neighbors
			if (bestOrigin->front == bestIntersection->front){
				Front& f = frontManager_[bestOrigin->front];

				if (f.PreviousElement(bestOrigin)->ID == bestIntersection->ID){
					bestNeighbor = bestOrigin;
					bestOrigin = f.NextElement(bestOrigin);
				}

				if (f.NextElement(bestOrigin)->ID == bestIntersection->ID){
					bestNeighbor = bestOrigin;
					bestOrigin = f.PreviousElement(bestOrigin);
				}
			}
				
			PostOrigin = bestOrigin->vertex.Position;
			PostAttempt = bestIntersection->vertex.Position;
		}
	}
}

bool MeshExtractor::IsBetterIntersection(const MeshExtractor::Intersection& intersection, const float& distance, const float& insideTriangleMinimum, const float& overallMinimum, const int& originalFront){
	/*
	How to decide if an intersection is better than previous ones:
	1) Prefer intersections inside the original triangle
	2) Prefer closer intersections
	3) Of intersections with the same distance, prefer thos which belong to our own front -> prefer spliting
	*/
	
	if (intersection.insideTriangle){
		if (distance < insideTriangleMinimum){
			return true;
		} else if (distance == insideTriangleMinimum) {
			return (intersection.edge.front == originalFront);
		} else {
			return false;
		}
	} else {
		// Stop if there has already been an intersection within the triangle
		if (insideTriangleMinimum != std::numeric_limits<float>::infinity()){
			return false;
		}

		if (distance < overallMinimum){
			return true;
		} else if (distance == overallMinimum) {
			return (intersection.edge.front == originalFront);
		} else {
			return false;
		}
	}
}

const Front::FrontListIterator& MeshExtractor::FindBestIntersectionElement(const MeshExtractor::Intersection& intersection, const MeshExtractor::Edge& originalEdge){
	/*
	How to decide which is the best front element for the intersection
	1) It mustn't be v1 or v2
	2) It has to be as close as possible
	3) The angle between it and the original edge must be smaller than 180° TODO: Good triangle shapes here?
	*/

	const Vector2f i(intersection.x, intersection.y);
	const Vector2f iv1(intersection.edge.v1.X(), intersection.edge.v1.Y());
	const Vector2f iv2(intersection.edge.v2.X(), intersection.edge.v2.Y());

	if (intersection.edge.frontIterator1->ID == originalEdge.frontIterator1->ID || intersection.edge.frontIterator1->ID == originalEdge.frontIterator2->ID){
		// 1 is within the original edge, check 2
		if (AngleIsValid(	Vector2f(originalEdge.v1.X(), originalEdge.v1.Y()), 
							Vector2f(originalEdge.v2.X(), originalEdge.v2.Y()), 
							iv2)){
			// Good intersection, pick it
			return intersection.edge.frontIterator2;
		} else {
			throw Exception();
		}
	} else if (intersection.edge.frontIterator2->ID == originalEdge.frontIterator1->ID || intersection.edge.frontIterator2->ID == originalEdge.frontIterator2->ID){
		// 2 is within the original edge, check 1
		if (AngleIsValid(	Vector2f(originalEdge.v1.X(), originalEdge.v1.Y()), 
							Vector2f(originalEdge.v2.X(), originalEdge.v2.Y()), 
							iv1)){
			// Good intersection, pick it
			return intersection.edge.frontIterator1;
		} else {
			throw Exception();
		}
	} else {
		// None of them is part of the original edge. Now only check distance and angle
		if (Length(i - iv1) < Length(i - iv2)){
			// Intersection1 is closer
			if (AngleIsValid(	Vector2f(originalEdge.v1.X(), originalEdge.v1.Y()), 
								Vector2f(originalEdge.v2.X(), originalEdge.v2.Y()), 
								iv1)){
				// Good intersection, pick it
				return intersection.edge.frontIterator1; 
			} else {
				return intersection.edge.frontIterator2; 
			}
		} else {
			// Intersection2 is closer
			if (AngleIsValid(	Vector2f(originalEdge.v1.X(), originalEdge.v1.Y()), 
								Vector2f(originalEdge.v2.X(), originalEdge.v2.Y()), 
								iv2)){
				// Good intersection, pick it
				return intersection.edge.frontIterator2; 
			} else {
				return intersection.edge.frontIterator1; 
			}
		}
	}
}

bool MeshExtractor::AngleIsValid(const Vector2f& origin, const Vector2f& v1, const Vector2f& v2){
	Vector2f x1 = origin - v1;
	Vector2f x2 = origin - v2;
	x1.Normalize();
	x2.Normalize();
	float degree = (Math::Atan2(x2.X(), x2.Y()) - Math::Atan2(x1.X(), x1.Y())).GetDegrees();
	return (degree < 180);
}

bool MeshExtractor::TestZCoordinate(const MeshExtractor::Edge& edge, const float acceptance){
	// We want to keep edges with coordinates that are within the acceptance or above and below 0.

	if (edge.v1.Z() >= 0 && edge.v2.Z() <= 0 || edge.v2.Z() >= 0 && edge.v1.Z() <= 0){
		return true;
	} else if (abs(edge.v1.Z()) <=  acceptance && abs(edge.v2.Z()) <=  acceptance){
		return true;
	}

	return false;
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

bool MeshExtractor::IsSame2DVertex(Vector2f& v1, Vector2f& v2){
	float floatingPointPrecision = 0.00001f;
	return v1.IsEqualEpsilon(v2, floatingPointPrecision);
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

	AdditionalDebugInfo.insert(AdditionalDebugInfo.begin(), frontManager_.AdditionalDebugInfo.begin(), frontManager_.AdditionalDebugInfo.begin() + frontManager_.AdditionalDebugInfo.size());
	frontManager_.AdditionalDebugInfo.clear();
}