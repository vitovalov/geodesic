/*
 *   Copyright (c) 2012
 *   This software and all its files are under GNU GPL v3.0 license. 
 *   See LICENSE.txt to read more about GNU GPL v3.0 license.
 *   Author: Vito Valov
 *   Developed with support of University of Barcelona
 *   Special thanks to Sergio Escalera & Albert Clapes
 */
/////////////////// DEFINES
#define SAMPLE_XML_PATH "......geodesic/Config.xml"
#define CHECK_RC(nRetVal, what) if (nRetVal != XN_STATUS_OK){printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));return nRetVal;}
#define MAXIMUM 10000000
#define INF (1<<20)
#define OPF_STEP 20
#define pair pair< int, int >
#define append(x) push_back(x)
#define FINFRAME 221
#define INIFRAME 220
#define BCK_COLOR 230