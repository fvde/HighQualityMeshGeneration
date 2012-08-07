#include "Front.h"


Front::Front()
{
	ID = -1;
	SeedFront = false;
}

Front::Front(int id)
	: ID(id)
{
	SeedFront = false;
}

void Front::Merge(const Front::FrontListIterator& ourMergePoint, Front::FrontListIterator& otherMergePoint, FrontManager& fm)
{
	Front& other = fm[otherMergePoint->front];
	// Check if we are merging with ourselves
	if (ourMergePoint->front == otherMergePoint->front){
		throw Exception();
	}

	// Make a copy to insert later
	Vertex copyOther = otherMergePoint->vertex;

	// Set the front references to our front
	for (auto it = other.Elements.begin(); it != other.Elements.end(); it++){
		it->front = ourMergePoint->front;
	}

    // Copy elements to our front (right before the merge point), from the merge point until the start of the other front
	Elements.splice(ourMergePoint, other.Elements, otherMergePoint, other.Elements.end());

    // Copy the rest
    Elements.splice(ourMergePoint, other.Elements, other.Elements.begin(), other.Elements.end());


	// Add a copy of the other's merge point to us, to complete the circle
	InsertVertex(copyOther, ourMergePoint, &fm);
	// TODO?
	otherMergePoint = PreviousElement(ourMergePoint);
	// Remove the other front
	fm.RemoveFront(other.ID);
}

void Front::Split(const FrontListIterator& origin, FrontListIterator& intersect, FrontManager& fm){

	if(origin->front != intersect->front){
		// Splitting different fronts
		throw Exception();
	}

	// New Front for the split
	int splitFrontID = fm.CreateFront();
	Front& f1 = fm[origin->front];
	Front& f2 = fm[splitFrontID];

	// Make a copy to insert later
	Vertex copyOrigin = origin->vertex;
	Vertex copyIntersect = intersect->vertex;

	// Find the cutstart and cutend
	FrontListIterator cutStart = intersect;
	FrontListIterator cutEnd = intersect;

	// Traverse the front until either the cut is found or we reach the start of our front
	while (cutEnd != f1.Elements.end() && cutEnd != origin) {
		cutEnd->front = splitFrontID;
		cutEnd++; 
    }

	// Copy all items we have found so far
	f2.Elements.splice(f2.Elements.end(), f1.Elements, cutStart, cutEnd);

	// If cutEnd is at the end of our front we still have to add the elements from our beginning until the origin
	if (cutEnd == Elements.end())
	{
		// So make another cut
		cutStart = cutEnd = f1.Elements.begin();

		while (cutEnd != f1.Elements.end() && cutEnd != origin)
		{
			cutEnd->front = splitFrontID;
			cutEnd++;
		}

		if (cutEnd == Elements.end())
		{
			// Didnt find the cut
			throw Exception();
		}

		// Copy the additional elements
		f2.Elements.splice(f2.Elements.end(), f1.Elements, cutStart, cutEnd);
	}

	// Because splice doesnt take the last element, origin is still in f1, but intersect is in f2
	// So we need to add a copy of origin to f2
	intersect = f2.Elements.begin();
	f2.InsertVertex(copyOrigin, intersect, &fm);

	// And we need to add a copy of intersect to f1
	f1.InsertVertex(copyIntersect, origin, &fm);
}

void Front::AddVertex(Vertex v, FrontManager* fm)
{
	FrontElement f;
	f.vertex = v;
	f.front = ID;
	f.fronts = fm;
	Elements.push_back(f);
}

Front::FrontListIterator Front::NextElement(const FrontListIterator it)
{
	FrontListIterator next = it;
	next++;
	if (next == Elements.end())
	{
		return Elements.begin();
	}
	return next;
}

Front::FrontListIterator Front::PreviousElement(const FrontListIterator it)
{
	FrontListIterator prev = it;
	if (prev == Elements.begin())
	{
		prev = Elements.end();
	}
	return --prev;
}

void Front::RemoveElement(FrontListIterator it)
{
	Elements.erase(it);
}

void Front::InsertVertex(Vertex element, FrontListIterator next, FrontManager* fm){
	FrontElement f;
	f.vertex = element;
	f.front = ID;
	f.fronts = fm;
	Elements.insert(next, f);
}

bool Front::IsEmpty(){
	return Elements.empty();
}

Front::FrontListIterator Front::LastElement(){
	return --Elements.end();
}

int Front::Size(){
	return Elements.size();
}

void Front::Clear(){
	Elements.clear();
}

Front::FrontListIterator Front::Random(){
	int element = Size() * ((float)rand() / RAND_MAX);
	FrontListIterator e = Elements.begin();

	while (element > 0){
		e = NextElement(e);
		element--;
	}

	return e;
}