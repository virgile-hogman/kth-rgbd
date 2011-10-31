// kth-rgbd: Visual SLAM from RGB-D data
// Copyright (C) 2011  Virgile Högman
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef CAMERA_DEVICE_H
#define CAMERA_DEVICE_H

#include "Display.h"
#include <vector>

class CameraDevice
{
private:
	bool		_abort;

public:	
	CameraDevice();

	bool connect();
	void disconnect();

	bool aborted()		{ return _abort; }

	bool generateFrame(int frameID, Display *display=NULL);
};

#endif // CAMERA_DEVICE_H
