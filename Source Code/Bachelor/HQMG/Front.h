#pragma once
/**
* @author  Ferdinand "Fred" von den Eichen
*
* License: NDL
*/

#include "Vertex.h"
#include <list>
#include <vector>
#include "niven.Core.Exception.h"
#include "niven.Core.PointerHelper.h"
#include <map>

using namespace niven;

class FrontManager;

class Front {

public:
	NIV_DEFINE_CLASS_SHARED_PTR(Front);
	
	struct FrontElement 
	{
		Vertex vertex;
		FrontManager* fronts;
		int	front;
		int ID;
	};

public:

	typedef std::list<FrontElement> FrontList;
	typedef FrontList::iterator FrontListIterator;

	Front();
	Front(int id);
	void Merge(const FrontListIterator& ourMergePoint, FrontListIterator& otherMergePoint, FrontManager& fm);
	void Split(const FrontListIterator& origin, FrontListIterator& intersect, FrontManager& fm); 
	// For initial elements only!
	void AddVertex(Vertex v, FrontManager* fm);
	FrontListIterator Random();
	FrontListIterator NextElement(const FrontListIterator it);
	FrontListIterator PreviousElement(const FrontListIterator it);
	void RemoveElement(FrontListIterator it);
	void InsertVertex(Vertex element, FrontListIterator next, FrontManager* fm);
	bool IsEmpty();
	FrontListIterator LastElement();
	int Size();
	void Clear();

public:
	FrontList Elements;
	int ID;
	bool SeedFront;
};

struct FrontManager
{
	FrontManager() { 
		currentlyHighestID = -1;
		currentlyHighestFrontElementID = -1;
		srand(0);
	};

	int CreateFront ()
	{
		currentlyHighestID++;
		const auto currentId = currentlyHighestID;
		fronts[currentId] = Front(currentId);
		return currentId;
	}

	void RemoveFront(int id){
		fronts.erase(id);
	}

	const int Count() const {
		return fronts.size();
	}

	std::map<int, Front>::iterator FrontsBegin(){
		return fronts.begin();
	}

	std::map<int, Front>::iterator FrontsEnd(){
		return fronts.end();
	}

	Front& GetRandomFront(){
		int element = fronts.size() * ((float)rand() / RAND_MAX);

		for (auto it = fronts.begin(); it != fronts.end(); it++, element--){
			if(element <= 0){
				return it->second;
			}
		}
		return fronts[0];
	}

	int GetFrontID(){
		currentlyHighestFrontElementID++;
		return currentlyHighestFrontElementID;
	}

	Front& operator [] (int id)
	{
		return fronts [id];
	}

	std::map<int, Front> fronts;
	int currentlyHighestID;
	int currentlyHighestFrontElementID;
};
