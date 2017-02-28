/*
 *   Copyright (c) 2012
 *   This software and all its files are under GNU GPL v3.0 license. 
 *   See LICENSE.txt to read more about GNU GPL v3.0 license.
 *   Author: Vito Valov
 *   Developed with support of University of Barcelona
 *   Special thanks to Sergio Escalera & Albert Clapes
 */
/////////////////// INCLUDES
// #include <GL/glut.h>
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
// #include "SceneDrawer.h"
#include <XnPropNames.h>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/image_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/pcd_io.h>

//DISJKSTRA
#include <cstdio>
#include <queue>
#include <vector>
#include <algorithm>

//OPTICAL FLOW
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>