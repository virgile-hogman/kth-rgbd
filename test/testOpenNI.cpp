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

#include "Config.h"
#include "CommonTypes.h"

/*#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "cv.h"
#include "highgui.h"*/

// Open NI
#include <XnCppWrapper.h>
#include <XnFPSCalculator.h>

using namespace xn;
using namespace std;

#define MAX_DEPTH_HISTOGRAM 10000		// previously used for histogram only

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
DepthGenerator g_depth;
ImageGenerator g_image;
DepthMetaData g_depthMD;
ImageMetaData g_imageMD;
Context g_context;
XnFPSData g_xnFPS;
XnUInt64 g_noSampleValue, g_shadowValue;

#define CHECK_RC(rc, what)                                            \
    if (rc != XN_STATUS_OK)                                            \
{                                                                \
	printf("%s failed: %s\n", what, xnGetStatusString(rc));        \
	return rc;                                                    \
}


// name of the configuration file where all the parameters are set
#define CONFIG_FILENAME	"config/kth-rgbd.cfg"

// -----------------------------------------------------------------------------------------------------
//  connect
// -----------------------------------------------------------------------------------------------------
bool connect()
{
	//Connect to kinect
	printf("Connecting to Kinect... ");
	fflush(stdout);
	XnStatus nRetVal = XN_STATUS_OK;
	EnumerationErrors errors;
	ScriptNode script;
	nRetVal = g_context.InitFromXmlFile(Config::_PathKinectXmlFile.c_str(), script, &errors);
	if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return false;
	}
	else if (nRetVal != XN_STATUS_OK)
	{
		printf("Open failed: %s\n", xnGetStatusString(nRetVal));
		return false;
	}
	printf("OK\n");

/*	// allocate the point cloud buffer
	g_cloudPointSave.width = NBPIXELS_WIDTH;
    g_cloudPointSave.height = NBPIXELS_HEIGHT;
    g_cloudPointSave.points.resize(NBPIXELS_WIDTH*NBPIXELS_HEIGHT);
*/
	nRetVal = g_context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_depth);
	CHECK_RC(nRetVal, "Find depth generator");

	nRetVal = g_context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_image);
	CHECK_RC(nRetVal, "Find image generator");

	nRetVal = xnFPSInit(&g_xnFPS, 180);
	CHECK_RC(nRetVal, "FPS Init");

	g_context.SetGlobalMirror(false); // mirror image horizontally

	g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);
	if (g_depth.GetIntProperty ("ShadowValue", g_shadowValue) != XN_STATUS_OK)
		printf ("[OpenNIDriver] Could not read shadow value!");

	if (g_depth.GetIntProperty ("NoSampleValue", g_noSampleValue) != XN_STATUS_OK)
		printf ("[OpenNIDriver] Could not read no sample value!");

    return (nRetVal == XN_STATUS_OK);
}

// -----------------------------------------------------------------------------------------------------
//  disconnect
// -----------------------------------------------------------------------------------------------------
void disconnect()
{
	g_context.Release();
}


void getFocalLength()
{
	// retro-engineering to read focal length from lib
	XnPoint3D in,out;
	in.X = 300;
	in.Y = 200;
	in.Z = 350;
	g_depth.ConvertProjectiveToRealWorld(1, &in, &out);
	printf("In \t= %f %f %f\n", in.X, in.Y, in.Z);
	printf("Out \t= %f %f %f\n", out.X, out.Y, out.Z);

	float focalInv = 1/525;
	float depth1 = in.Z;
	float x1 = (in.X - NBPIXELS_X_HALF) * depth1 * focalInv;
	float y1 = (NBPIXELS_Y_HALF - in.Y) * depth1 * focalInv;
	float z1 = depth1 ; // given depth values are in mm
	printf("Out \t= %f %f %f\n", x1, y1, z1);

	double focalx = NBPIXELS_WIDTH  / (out.X / ((in.X/NBPIXELS_WIDTH - 0.5)*depth1));
	double focaly = NBPIXELS_HEIGHT  / (out.Y / ((in.Y/NBPIXELS_HEIGHT - 0.5)*depth1));
	printf("fx \t= %lf\n", focalx);
	printf("fy \t= %lf\n", focaly);

	//in.X = 432;
	//in.Y = 129;
	//in.Z = 378;
	g_depth.ConvertProjectiveToRealWorld(1, &in, &out);
	printf("In \t= %f %f %f\n", in.X, in.Y, in.Z);
	printf("Out \t= %f %f %f\n", out.X, out.Y, out.Z);

	depth1 = in.Z;
	x1 = (in.X - NBPIXELS_X_HALF) * depth1 * (1/focalx);
	y1 = (NBPIXELS_Y_HALF - in.Y) * depth1 * (1/focaly);
	z1 = depth1 ; // given depth values are in mm
	printf("Out \t= %f %f %f\n", x1, y1, z1);
}


// -----------------------------------------------------------------------------------------------------
//  Main program
// -----------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
	// load configuration
	Config::LoadConfig(CONFIG_FILENAME);
    //FrameData::_DataPath = Config::_PathFrameSequence;

	srand(time(NULL));

	if (connect()) {
		getFocalLength();
		disconnect();
	}
}
