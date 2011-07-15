#ifndef MAP_H
#define MAP_H

#include <vector>

class Map
{
public:
		
	void buildFromSequence(std::vector<int> &sequenceFramesID);
	
	void buildFromArchive();

};

#endif // MAP_H

