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

	bool generateFrame(int &frameID);
};

#endif // CAMERA_DEVICE_H
