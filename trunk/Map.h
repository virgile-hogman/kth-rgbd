#ifndef MAP_H
#define MAP_H

#include <vector>

class Map
{
public:
		
	void buildFromSequence(std::vector<int> &sequenceFramesID, bool savePointCloud);
	
	void buildFromArchive();

};

#endif // MAP_H

