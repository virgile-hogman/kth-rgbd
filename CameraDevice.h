#ifndef CAMERA_DEVICE_H
#define CAMERA_DEVICE_H

#include <vector>

class CameraDevice
{
public:
	static float _FocalLength;
	
public:	
	bool connect();
	void disconnect();

	void generateFrames(int nbRemainingFrames, std::vector<int> &framesID);
};

#endif // CAMERA_DEVICE_H