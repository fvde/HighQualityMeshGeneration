#include "MeshExtractor.h"



MeshExtractor::MeshExtractor(const ShardFileParser::Ptr& sfp, float isoValue, float p, float n)
	: sfp_(sfp),
	isoValue_(isoValue){

		surface_  = SplineSurface::Ptr (new SplineSurface(sfp, isoValue));
		guidanceField_ = GuidanceField::Ptr ( new GuidanceField(sfp, surface_, p, n));
		indiceCounter_ = 0;
		debugMaxTriangleNumber_ = 75;
		debugTriangleCounter_ = debugMaxTriangleNumber_;
		totalSteps_ = 0;
		floatingPointPrecision_ = 0.0001f;
		// TODO Adaptive z coodinate acceptance, taking into account local curvature
		collisionZCoordinateAcceptance_ = 0.2f;
		CurrentZAcceptance = collisionZCoordinateAcceptance_;
		projectionTolerance_ = 0.25f;
		debugMode_ = false;
		meshColor_ = Vector4f(0.0f, 0.5f, 1.0f, 1.0f);
	}

void MeshExtractor::Setup(){	
	// TODO:: Find more/better seeds
	int size = guidanceField_->GetSamples().size();
	int seedNumber = ((float)rand() / RAND_MAX) * size;
	seedNumber = Min(seedNumber, size - 1);
	Vector3f seed = guidanceField_->GetSamples()[seedNumber].position;

	// Find a starting front
	GetFrontFromSeed(seed, frontManager_);

	if (!debugMode_){
		Log::DisableLogging ("MeshExtractor");
	}
}


void MeshExtractor::TriangulateShardFile(std::vector<Vector3f>& vertices, std::vector<Vector3f>& normals, std::vector<Vector4f>& colors, std::vector<uint32>& indices, int steps){

	totalSteps_ += steps;
	int currentRequestTotalSteps = steps;

	char buffer[200];
	sprintf(buffer, "Executing %i steps. Total number of steps is %i", steps, totalSteps_);
	Log::Info("Triangulator", buffer);

	// Vector for finished triangles
	std::vector<Triangle> triangles;

	// Start the algorithm
	while (frontManager_.Count() > 0 && (steps > 0)){
		steps--;
		Front& currentFront = frontManager_.GetRandomFront();
		int currentFrontID = currentFront.ID;

		// Debugging
		DisplayProgress(steps, currentRequestTotalSteps);

		// Remove if invalid
		if (currentFront.ID == -1){
			frontManager_.RemoveFront(currentFrontID);
			continue;
		}

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
			Log::Info("MeshExtractor", "Vertex creation failed!");
			continue;
		}

		if (!ProjectVertexOnSurface(attemptVertex, projectionTolerance_)){
			Log::Info("MeshExtractor", "Projection failed!");
			continue;
		}

		// The vertex is valid, so update its information
		attemptVertex.Normal = surface_->GetNormal(attemptVertex.Position);

		// Update Guidance field information for the new vertex
		attemptVertex.IdealEdgeLength = guidanceField_->Evaluate(attemptVertex.Position);
		attemptVertex.Color = meshColor_;

		// Update debug information
		PreAttempt = attemptVertex.Position;
		PostAttempt = attemptVertex.Position;

		// Declare for identification of interfering fronts (by reference)
		Front::FrontListIterator bestIntersection;
		bool testFailed;

		if(!CheckFrontInterference(attemptVertex, bestOrigin, bestNeighbor, bestIntersection, testFailed))
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
			if (testFailed){
				Log::Info("MeshExtractor", "Front interference test was not able to determine a viable intersection!");
				continue;
			}

			if (currentFrontID == bestIntersection->front){
				// Split fronts ( -> Creates a new front)
				currentFront.Split(bestOrigin, bestIntersection, frontManager_);
			} else {
				// Merge fronts ( -> Removes a front)

				if (frontManager_[bestIntersection->front].Size() <= 3){
					Log::Info("MeshExtractor", "Didn't merge, because the other front only contains 3 elements!");
					continue;
				}
				currentFront.Merge(bestOrigin, bestIntersection, frontManager_);
			}
		}
	}

	char buffer2[200];
	sprintf(buffer2, "Steps completed. Generated %i triangles.", triangles.size());
	Log::Info("Triangulator", buffer2);

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
	float edgeLength = guidanceField_->Evaluate(seed);

	Vector3f n = surface_->GetNormal(seed);
	Vector3f n2 = Vector3f(n.Y(), n.X(), n.Z());
	Vector3f surfaceVector = Cross(n, n2);

	int newfrontID = fm.CreateFront();

	Vertex v1 = Vertex(seed + 0.5 * edgeLength * surfaceVector);
	Vertex v2 = Vertex(seed - 0.5 * edgeLength * surfaceVector);

	v1.Normal = surface_->GetNormal(v1.Position);
	v2.Normal = surface_->GetNormal(v2.Position);

	v1.IdealEdgeLength = guidanceField_->Evaluate(v1.Position);
	v1.Color = meshColor_;

	v2.IdealEdgeLength = guidanceField_->Evaluate(v2.Position);
	v2.Color = meshColor_;

	fm[newfrontID].AddVertex(v1, &fm);
	fm[newfrontID].AddVertex(v2, &fm);
	fm[newfrontID].SeedFront = true;
}

bool MeshExtractor::GrowVertex(Vertex& v1, Vertex& v2, Vertex& vNew)
{
	// TODO
	if (v1.Position == v2.Position){
		return false;
	}

	Vector3f edgeDir = v2.Position - v1.Position;
	const float distance = Length(edgeDir);

	// Calculate vNew's position

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

	// We need the edges normal for projection, this is NOT the final normal of the vertex!
	vNew.Normal = edgeNormal;

	return true;
}

bool MeshExtractor::ProjectVertexOnSurface(Vertex& v, const float& tolerance){
	
	// TODO Projection in direction of the normal of v! For now attempt all directions and pick the closest result. :/
	std::vector<Vector3f> projections;
	Vector3f originalPosition = v.Position;

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

	// Additionally to Newton use another projection operator, then pick the best result
	Vector3f newPosition;
	bool success = surface_->ProjectOnSurface(v.Position, v.Normal, newPosition);

	if (success){
		projections.push_back(newPosition);
	}

	if(projections.empty()){
		return false;
	}

	v.Position = (*std::min_element(projections.begin(), projections.begin() + projections.size(), [&v] (const Vector3f pos1, const Vector3f pos2) -> bool {
		return Length(pos1-v.Position) < Length(pos2-v.Position);
	}));

	// Check if the projection was successful
	if (v.Position.X() != v.Position.X() || v.Position.Y() != v.Position.Y() || v.Position.Z() != v.Position.Z()){
		// Undefined position!
		return false;
	}

	// Check if the projected position is within reasonable distance
	if (Length(v.Position - originalPosition) > tolerance){
		return false;
	}

	return true;
}

bool MeshExtractor::CheckFrontInterference(Vertex newVertex, Front::FrontListIterator& bestOrigin, Front::FrontListIterator& bestNeighbor, Front::FrontListIterator& bestIntersection, bool& interferenceTestFailed){

	// Create the edges of the new triangle
	Edge intersectionOrigin(newVertex, bestOrigin);
	Edge intersectionNeighbor(newVertex, bestNeighbor);
	Edge originNeighbor(bestOrigin, bestNeighbor);
	interferenceTestFailed = false;

	// Debugging
	PreOrigin = intersectionOrigin.v2;
	PreNeighbor = intersectionNeighbor.v2;
	PreAttempt = intersectionOrigin.v1;
	TransformedPostOrigin = Vector3f(0, 0, 0);
	TransformedPostNeighbor = Vector3f(0, 0, 0);
	TransformedPostAttempt = Vector3f(0, 0, 0);
	LastEdgesInRange.clear();
	Intersections.clear();

	/* 
	Create a coordinate system that is spanned by the vector pointing from the 
	new vertex to the current vertex and a vector orthogonal to that.	
	In this space we will do all the intersection detection
	*/
	Vector3f originOffset;
	Matrix3f coordinateSystem;
	Get2DTransformation(intersectionOrigin, intersectionNeighbor, originOffset, coordinateSystem); 

	/*
	Now we will test this triangle against an appropriate 2D area around it.
	Intersections will be:
	1) Direct edge/edge intersections
	2) Points inside the triangle (because our triangle could include a geometric feature completely)
	*/
	std::vector<MeshExtractor::Intersection> intersections;
	TestTriangle(	intersectionOrigin,
					intersectionNeighbor,
					originNeighbor,
					newVertex,
					originOffset,
					coordinateSystem,
					intersections,
					false);

	// If we didn't find any intersections we're done.
	if (intersections.size() == 0){
		return false;
	}

	/*
	Now we only have to find the intersection that fits best.
	Several criteria have to be met
	1) The intersection may not be part of the triangle or directly adjacent
	2) The angle between the original edge and the new edges must always be in [0,180°]
	3) Inside triangle points will be prefered over outside triangle points (conservative growth)
	4) Splitting is prefered over merging
	*/

	if (!FindBestIntersection(intersections, originNeighbor, bestOrigin, bestNeighbor, bestIntersection)){
		// None of the intersections was viable, abort.
		interferenceTestFailed = true;
		return true;
	}

	/* 
	We have found the ideal intersection and origin. But this intersection could 
	lead to new intersections if we're unlucky. The new triangle we have to test 
	consists of the edge originNeighbor and the best intersection
	*/
	int safetyCounter = 5;
	do {
		safetyCounter--;

		Edge newIntersectionOrigin(bestIntersection, bestOrigin);
		Edge newIntersectionNeighbor(bestIntersection, bestNeighbor);
		Edge newOriginNeighbor(bestOrigin, bestNeighbor);

		intersections.clear();
		TestTriangle(	newIntersectionOrigin,
						newIntersectionNeighbor,
						newOriginNeighbor,
						bestIntersection->vertex,
						originOffset,
						coordinateSystem,
						intersections,
						true);

		if (intersections.size() > 0) {
			if (!FindBestIntersection(intersections, newOriginNeighbor, bestOrigin, bestNeighbor, bestIntersection)){
				// None of the intersections was viable, abort.
				interferenceTestFailed = true;
				return true;
			}
		}

	} while (intersections.size() > 0 && safetyCounter > 0);

	// Check if we found a triangle that didnt lead to any new intersections
	if (intersections.size() > 0){
		// Failed to find such a triangle
		interferenceTestFailed = true;
		return true;
	}

	return true;
}

void MeshExtractor::TestTriangle(MeshExtractor::Edge& intersectionOrigin, MeshExtractor::Edge& intersectionNeighbor, MeshExtractor::Edge& originNeighbor, const Vertex& newVertex, const Vector3f& translation, const Matrix3f& transformation, std::vector<MeshExtractor::Intersection>& intersections, const bool& isSecondaryTest){
	
	std::vector<MeshExtractor::Edge> edges = GetEdgesInRange(newVertex, originNeighbor);

	// Translate and transform
	TransformEdge(intersectionOrigin, -translation, transformation);
	TransformEdge(intersectionNeighbor, -translation, transformation);
	TransformEdge(originNeighbor, -translation, transformation);

	// Debugging
	if(!isSecondaryTest){
		TransformedPreOrigin = Vector3f(intersectionOrigin.v2.X(), intersectionOrigin.v2.Y(), 0);
		TransformedPreNeighbor = Vector3f(intersectionNeighbor.v2.X(), intersectionNeighbor.v2.Y(), 0);
		TransformedPreAttempt = Vector3f(intersectionOrigin.v1.X(), intersectionOrigin.v1.Y(), 0);
	}

	for (auto it = edges.begin(); it != edges.end(); it++){
		TransformEdge((*it), -translation, transformation);
		LastEdgesInRange.push_back(*it);
	}

	// Test the two new edges against all other found edges
	TestEdge(intersectionOrigin, edges, intersections);
	TestEdge(intersectionNeighbor, edges, intersections);

	// Test if any of the points of the found edges is within the triangle
	MeshExtractor::Intersection intersection;
	intersection.origin = originNeighbor.frontIterator1;
	for (auto it = edges.begin(); it != edges.end(); it++){

		intersection.edge = (*it);
		FindPointWithinTriangle(intersection, 
								Vector2f(intersectionOrigin.v1.X(), intersectionOrigin.v1.Y()),
								Vector2f(intersectionOrigin.v2.X(), intersectionOrigin.v2.Y()),
								Vector2f(intersectionNeighbor.v2.X(), intersectionNeighbor.v2.Y()),
								intersections);

	}

	// If its a secondary test, remove the intersection with the bestIntersection vertex
	if (isSecondaryTest) {

		std::vector<MeshExtractor::Intersection> filteredIntersections;
		Vector2f i = Vector2f(intersectionOrigin.v1.X(), intersectionOrigin.v1.Y());

		for (auto it = intersections.begin(); it != intersections.end(); it++) {
			if (!IsSame2DVertex(Vector2f(it->x, it->y), i))                                                         
			{                                                                       
				filteredIntersections.push_back(*it);
			}                                                                       
		}

		intersections = filteredIntersections;
	}
}

void MeshExtractor::Get2DTransformation(const MeshExtractor::Edge& edge1, const MeshExtractor::Edge& edge2, Vector3f& translation, Matrix3f& transformation){
		
	// The translation is the offset to the origin. Which is the intersection point (the new vertex).
	translation = edge1.v1;
	
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
	transformation = Matrix3f(	u.X(), u.Y(), u.Z(),
								v.X(), v.Y(), v.Z(),
								n.X(), n.Y(), n.Z());

}

std::vector<MeshExtractor::Edge> MeshExtractor::GetEdgesInRange(const Vertex& v, const MeshExtractor::Edge& edgeCurrentPrevious){
	std::vector<MeshExtractor::Edge> elementsInRange;

	// Ideal edge length has ben calculated before. Factor 3 because vertices could be outside the ideal length sphere and still intersect
	float range = v.IdealEdgeLength * 3;

	// TODO: Octree or any other spatial subdivision for faster traversal!
	for (auto frontIDIt = frontManager_.FrontsBegin(); frontIDIt != frontManager_.FrontsEnd(); frontIDIt++){
		Front& f = frontIDIt->second;

		for (auto frontElementIt = f.Elements.begin(); frontElementIt != f.Elements.end(); frontElementIt++){
			if (Length(v.Position - frontElementIt->vertex.Position) < range){
				// Make a copy and push back
				MeshExtractor::Edge e(frontElementIt, f.PreviousElement(frontElementIt));

				// Dont use if its the original edge
				if ((e.frontIterator1->ID != edgeCurrentPrevious.frontIterator1->ID 
					&& e.frontIterator2->ID != edgeCurrentPrevious.frontIterator2->ID)
					||(e.frontIterator1->ID != edgeCurrentPrevious.frontIterator2->ID 
					&& e.frontIterator2->ID != edgeCurrentPrevious.frontIterator1->ID)){
					elementsInRange.push_back(e);
				}
			}
		}
	}

	return elementsInRange;
}

void MeshExtractor::TestEdge(const MeshExtractor::Edge& edge, const std::vector<MeshExtractor::Edge>& edgesInRange, std::vector<MeshExtractor::Intersection>& intersections){

	// The Edge MUST by transformed into the adequate 2D space beforehand!

	MeshExtractor::Intersection currentIntersection;

	// Other elements
	for (auto it = edgesInRange.begin(); it != edgesInRange.end(); it++){
		const Edge& e = *it;
		LastEdgesInRange.push_back(e);

		// Discard edges which are not intersecting our slice of the mesh
		if(!TestZCoordinate(e, collisionZCoordinateAcceptance_)){
			continue;
		}
		
		// Edges with undefined attributes will be true	
		if (e.v2.X() != e.v2.X() || e.v2.Y() != e.v2.Y()){
			int fail = 0;
		}

		// Test Edge
		if (GetIntersection(edge, e, currentIntersection)){
			intersections.push_back(currentIntersection);
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

		Vector2f intersect(intersection.edge.v1.X(), intersection.edge.v1.Y());
		i.x = intersect.X();
		i.y = intersect.Y();
		i.insideTriangle = true;

		if ((!IsSame2DVertex(intersect, v1)) && (!IsSame2DVertex(intersect, v2)) && (!IsSame2DVertex(intersect, v3)))
		{
			Intersections.push_back(Vector3f(i.x, i.y, 0));
			intersections.push_back(i);
		}
	} 
	else  if (MeshExtractor::IsInsideTriangle(v1, v2, v3, Vector2f(intersection.edge.v2.X(), intersection.edge.v2.Y()))){
		Intersection i(intersection);
		
		Vector2f intersect(intersection.edge.v2.X(), intersection.edge.v2.Y());
		i.x = intersect.X();
		i.y = intersect.Y();
		i.insideTriangle = true;

		if ((!IsSame2DVertex(intersect, v1)) && (!IsSame2DVertex(intersect, v2)) && (!IsSame2DVertex(intersect, v3)))
		{
			Intersections.push_back(Vector3f(i.x, i.y, 0));
			intersections.push_back(i);
		}
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

bool MeshExtractor::FindBestIntersection(const std::vector<MeshExtractor::Intersection>& intersections, const MeshExtractor::Edge& originalEdge, Front::FrontListIterator& bestOrigin, Front::FrontListIterator& bestNeighbor, Front::FrontListIterator& bestIntersection){

	// We now have a list of edges interfering with us.
	// v1 := originalEdge.v1 v2:= originalEdge.v2
	// The final intersecion should fullfill the following criteria:
	// 1) It shouldn't be an intersection with v1 or v2: Guaranteed by GetIntersection.
	// 2) Its distance to v1 and v2 should be minimal.
	// 3) The edges between the intersection vertex and v1 and v2 should not lead to new intersections.
	// We test 3) later

	const Vector2f v1(originalEdge.v1.X(), originalEdge.v1.Y());
	const Vector2f v2(originalEdge.v2.X(), originalEdge.v2.Y());

	bool intersectionFound = false;

	float currentMinimalDistance = std::numeric_limits<float>::infinity();
	float currentInsideMinimalDistance = currentMinimalDistance;
	float currentSplitMinimalDistance = currentMinimalDistance;

	for (auto it = intersections.begin(); it != intersections.end(); it++){
			
		// Current intersection
		Vector2f intersection(it->x, it->y);
		Vector2f intersectionv1(it->edge.v1.X(), it->edge.v1.Y());
		Vector2f intersectionv2(it->edge.v2.X(), it->edge.v2.Y());

		// Note that originalEdge has been transformed into our 2D world.
		float accumulatedDistance = Length(intersection - v1) + Length(intersection - v2);

		// Decide if its a good intersection to pick
		if (IsBetterIntersection((*it), accumulatedDistance, currentInsideMinimalDistance, currentSplitMinimalDistance, currentMinimalDistance, originalEdge.front)) 
		{

			// Find the vertex of the edge that is not v1 or v2 and closest to the intersection
			if (!FindBestIntersectionElement((*it), originalEdge, bestIntersection)){
				// The intersection was not useable! This is critical.
				Log::Info("MeshExtractor", "No intersection element was viable!");
				continue;
			} else {
				intersectionFound = true;
			}
			bestOrigin = it->origin;

			// This intersection is a minimal candidate. 
			currentMinimalDistance = accumulatedDistance;

			if (it->insideTriangle){
				currentInsideMinimalDistance = accumulatedDistance;
			}

			if (it->edge.front == originalEdge.front){
				currentSplitMinimalDistance = accumulatedDistance;
			}

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

			// Debugging
			if (bestIntersection->ID == it->edge.frontIterator1->ID){
				TransformedPostAttempt = Vector3f(it->edge.v1.X(), it->edge.v1.Y(), 0);
			} else {
				TransformedPostAttempt = Vector3f(it->edge.v2.X(), it->edge.v2.Y(), 0);
			}
	
			PostAttempt = bestIntersection->vertex.Position;
			PostOrigin = bestOrigin->vertex.Position;
			PostNeighbor = bestNeighbor->vertex.Position;
		}
	}
	
	// Debugging
	TransformedPostOrigin = Vector3f(originalEdge.v1.X(), originalEdge.v1.Y(), 0);
	TransformedPostNeighbor = Vector3f(originalEdge.v2.X(), originalEdge.v2.Y(), 0);

	return intersectionFound;
}


bool MeshExtractor::IsBetterIntersection(const MeshExtractor::Intersection& intersection, const float& distance, const float& insideTriangleMinimum, const float& splitMinimum, const float& overallMinimum, const int& originalFront){
	
	// How to decide if an intersection is better than previous ones:
	// 1) Prefer intersections inside the original triangle
	// 2) Prefer closer intersections
	// 3) Of intersections with the same distance, prefer thos which belong to our own front -> prefer spliting
	
	
	if (intersection.insideTriangle){
		if (distance < insideTriangleMinimum){
			return true;
		} else if (abs(distance - insideTriangleMinimum) < floatingPointPrecision_) {
			return (intersection.edge.front == originalFront);
		} else {
			return false;
		}
	} else if (intersection.edge.front == originalFront){
		if (distance < splitMinimum){
			return true;
		} else {
			return false;
		}
	} else {
		// Stop if there has already been a better intersection within the triangle
		if (insideTriangleMinimum != std::numeric_limits<float>::infinity() 
			|| splitMinimum != std::numeric_limits<float>::infinity()){
			return false;
		}

		if (distance < overallMinimum){
			return true;
		} else {
			return false;
		}
	}
}

/*

bool MeshExtractor::IsBetterIntersection(const MeshExtractor::Intersection& intersection, const float& distance, const float& insideTriangleMinimum, const float& splitMinimum, const float& overallMinimum, const int& originalFront){

	// How to decide if an intersection is better than previous ones:
	// 1) Prefer intersections inside the original triangle
	// 2) Prefer closer intersections
	// 3) Of intersections with the same distance, prefer thos which belong to our own front -> prefer spliting
	
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

*/

bool MeshExtractor::FindBestIntersectionElement(const MeshExtractor::Intersection& intersection, const MeshExtractor::Edge& originalEdge, Front::FrontListIterator& bestElement){
	/*
	How to decide which is the best front element for the intersection
	1) It mustn't be v1 or v2
	2) It has to be as close as possible
	3) The angle between it and the original edge must be smaller than 180° TODO: Good triangle shapes here?
	*/

	const Vector2f i(intersection.x, intersection.y);
	const Vector2f iv1(intersection.edge.v1.X(), intersection.edge.v1.Y());
	const Vector2f iv2(intersection.edge.v2.X(), intersection.edge.v2.Y());
	const Vector2f original1(originalEdge.v1.X(), originalEdge.v1.Y());
	const Vector2f original2(originalEdge.v2.X(), originalEdge.v2.Y());

	// ID's for better access and readability of code
	const int intersectionID1 = intersection.edge.frontIterator1->ID;
	const int intersectionID2 = intersection.edge.frontIterator2->ID;
	const int originID1 = originalEdge.frontIterator1->ID;
	const int originID2 = originalEdge.frontIterator2->ID;


	if (intersectionID1 == originID1 || intersectionID1 == originID2){
		// 1 is within the original edge, check 2
		if (AngleIsValid(	original1, 
							original2, 
							iv2)){
			// Good intersection, pick it
			bestElement = intersection.edge.frontIterator2;
		} else {
			return false;
		}
	} else if (intersectionID2 == originID1 || intersectionID2 == originID2){
		// 2 is within the original edge, check 1
		if (AngleIsValid(	original1, 
							original2, 
							iv1)){
			// Good intersection, pick it
			bestElement = intersection.edge.frontIterator1;
		} else {
			return false;
		}
	} else {
		// None of them is part of the original edge. Now only check distance and angle
		if (Length(i - iv1) < Length(i - iv2)){
			// Intersection1 is closer
			if (AngleIsValid(	original1, 
								original2, 
								iv1)){
				// Good intersection, pick it
				bestElement =  intersection.edge.frontIterator1; 
			} else {
				bestElement = intersection.edge.frontIterator2; 
			}
		} else {
			// Intersection2 is closer
			if (AngleIsValid(	original1, 
								original2, 
								iv2)){
				// Good intersection, pick it
				bestElement = intersection.edge.frontIterator2; 
			} else {
				bestElement = intersection.edge.frontIterator1; 
			}
		}
	}

	return true;
}

bool MeshExtractor::AngleIsValid(const Vector2f& origin, const Vector2f& dest, const Vector2f& point){
	float value = ((dest.X() - origin.X())*(point.Y() - origin.Y())) - ((dest.Y() - origin.Y())*(point.X() - origin.X()));
	return (value < 0);
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

bool MeshExtractor::IsSame2DVertex(const Vector2f& v1, const Vector2f& v2){
	return v1.IsEqualEpsilon(v2, floatingPointPrecision_);
}

void MeshExtractor::ParseFrontManager(){
	DebugFront.clear();

	for (auto frontIDIt = frontManager_.FrontsBegin(); frontIDIt != frontManager_.FrontsEnd(); frontIDIt++){
		Front& f = frontIDIt->second;
		std::vector<Vector3f> front;

		if (f.ID == -1){
			continue;
		}

		for (auto frontElementIt = f.Elements.begin(); frontElementIt != f.Elements.end(); frontElementIt++){
			front.push_back(frontElementIt->vertex.Position);
		}

		DebugFront.push_back(front);
	}

	AdditionalDebugInfo.insert(AdditionalDebugInfo.begin(), frontManager_.AdditionalDebugInfo.begin(), frontManager_.AdditionalDebugInfo.begin() + frontManager_.AdditionalDebugInfo.size());
	frontManager_.AdditionalDebugInfo.clear();
}

void MeshExtractor::DisplayProgress(const int& current, const int& total){
	if (total < 10){
		return;
	} else {
		int tenPercent = total/10;
		if (current % tenPercent == 0){
			char buffer[200];
			sprintf(buffer, "Triangulating... (%i %% done)", (int)(100.0f * (1 - current/(float)total)));
			Log::Info("Triangulator", buffer);
		}
	}
}

std::vector<GuidanceFieldSample> MeshExtractor::GetSamples(){
	return guidanceField_->GetSamples();
}