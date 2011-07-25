#ifndef MAP_H
#define MAP_H

#include <vector>

class Map
{
public:
		
	void buildFromSequence(std::vector<int> &sequenceFramesID);
	
	void buildFromArchive();

	void regeneratePCD();

};

#endif // MAP_H

