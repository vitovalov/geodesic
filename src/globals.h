/*
 *   Copyright (c) 2012
 *   This software and all its files are under GNU GPL v3.0 license. 
 *   See LICENSE.txt to read more about GNU GPL v3.0 license.
 *   Author: Vito Valov
 *   Developed with support of University of Barcelona
 *   Special thanks to Sergio Escalera & Albert Clapes
 */
#include "main.h"
//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------

xn::Context g_Context;
xn::ScriptNode g_scriptNode;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator g_UserGenerator;
xn::ImageGenerator g_ImageGenerator;
xn::DepthMetaData depthMD;
xn::SceneMetaData sceneMD;
xn::ImageMetaData imageMD;
xn::Recorder recorder;
xn::Player g_Player;
XnStatus nRetVal = XN_STATUS_OK;

static const double pi = 3.14159265358979323846;
using namespace pcl;
typedef PointXYZRGB PointT;
PointCloud<PointT>::Ptr pCloud (new PointCloud<PointT>);
PointCloud<PointT>::Ptr pCloudFiltered (new PointCloud<PointT>);
PointCloud<PointT>::Ptr cloud_out (new PointCloud<PointT>);
pcl::VoxelGrid< PointT > vxGrid;
int frameID;
Mat imageA;
Mat imageB;
int xRes=640,yRes=480;
int fileNumber = 0;
float NaN = numeric_limits<float>::quiet_NaN();
