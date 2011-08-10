#ifndef MAP_H
#define MAP_H

#include <vector>

class Map
{
public:
		
	void buildFromSequence(std::vector<int> &sequenceFramesID);
	
	void buildFromArchive(int minFrameID, int maxFrameID);

	void regeneratePCD();

};

#endif // MAP_H

