#ifndef CAMERA_DEVICE_H
#define CAMERA_DEVICE_H

#include <vector>

class CameraDevice
{
public:
	static float _FocalLength;
	
private:
	bool		_abort;

public:	
	CameraDevice();

	bool connect();
	void disconnect();

	bool aborted()		{ return _abort; }

	bool generateFrame(int frameID);
};

#endif // CAMERA_DEVICE_H
