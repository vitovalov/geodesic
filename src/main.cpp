/*
 *   Copyright (c) 2012
 *   This software and all its files are under GNU GPL v3.0 license. 
 *   See LICENSE.txt to read more about GNU GPL v3.0 license.
 *   Author: Vito Valov
 *   Developed with support of University of Barcelona
 *   Special thanks to Sergio Escalera & Albert Clapes
 */
#include "main.h"

using namespace cv;
using namespace pcl;
using namespace std;
#include "globals.h"
#include "defines.h"


struct kinectData{
    xn::DepthMetaData depthMD;
    xn::SceneMetaData sceneMD;
    xn::Context g_Context;
    xn::ScriptNode g_scriptNode;
    xn::DepthGenerator g_DepthGenerator;
    xn::UserGenerator g_UserGenerator;
    xn::ImageGenerator g_ImageGenerator;
    xn::ImageMetaData imageMD;
    xn::Recorder recorder;
    xn::Player g_Player;
    XnStatus nRetVal = XN_STATUS_OK;
};
/**
 * structure for dijkstra
 */
struct Indist{
    vector<int> neighborIndices;//TODO make linkedlist
    vector<int> distances;
};
/**
 * distance comparator
 */
struct comp{
    bool operator() (const pairInts &a, const pairInts &b){
        return a.second > b.second;
    }
};

struct valPoint{
    float v;
    pair<XnPoint3D,XnPoint3D> par;
    
};
struct MatVec{
    Mat matr;
    vector<float> vec;
    vector<pairInts > hands;
};

/**
 * finds neighbours to certain point in pointcloud
 * @param  idx index of point in structure
 * @return     struct Indist
 */
Indist getNeighborIndices(int idx) {
    PointCloud<PointT>::Ptr pc = pCloudFiltered;
    int eucDist = 0;
    Indist ret;
    ret.neighborIndices.clear();
    ret.distances.clear();
    int u = 0;
    int nindex;
    int zVAL = 0; //value of z coordinate pQ
    
    // A point to start in real world coords.
    const pcl::PointXYZRGB pF = pc->points[idx];
    if(pF.z == 0  ){
        //        printf("getNeighborIndices:NaN\t");
    }else{
        // Coordinates of p in a discrete grid space, expressed as vector = (i,j,k).
        Eigen::Vector3i pgrid = vxGrid.getGridCoordinates(pF.x, pF.y, pF.z); // x,y,z -> i,j,k
        // printf("coordenadas cabeza voxgrid:%f,%f,%f\n",pF.x,pF.y,pF.z);
        // Move around the relative voxels.
        
        for (int i = -1; i <= 1; i++)
            for (int j = -1; j <= 1; j++)
                for (int k = -1; k <= 1; k++)
                {
                    
                    Eigen::Vector3i pneighborgrid = pgrid + Eigen::Vector3i(i,j,k); // Get certain neighbor.
                    nindex = vxGrid.getCentroidIndexAt(pneighborgrid); // Get the centroid of the neighbor.
                    
                    
                    if (nindex >= 0 && nindex != idx) // Index -1 indicates there is no neighbour (bound)
                    {
                        // //printf("nindex:%d\n",nindex  );
                        // Print the index and the neighbor point z value.
                        // //cout << "nindex: " << nindex << ", z: " <<  pCloudFiltered->points[nindex].z << endl;
                        
                        // pCloudFiltered->points[nindex].r = 50;
                        // pCloudFiltered->points[nindex].g = 255;
                        // pCloudFiltered->points[nindex].b = 50;
                        const pcl::PointXYZRGB pQ = pc->points[nindex];
                        if(pQ.z != zVAL){
                            eucDist = sqrtf(powf((pF.x-pQ.x),2)+powf((pF.y-pQ.y),2)+powf((pF.z-pQ.z),2));
                            // printf("new eucDist = %d\n",eucDist);
                            ret.neighborIndices.push_back(nindex);
                            //printf("eucDist:%d\n",eucDist);
                            ret.distances.push_back(eucDist);
                            u++;
                        }else{
                            // printf("z infinito!\n");
                        }
                        //cout << "nindex="<< nindex << "; Distance = " << eucDist << endl;
                    }
                    
                }
    }
    return ret;
};
inline static double square(int a)
{
    return a * a;
}

/*------------------------------------*
 *          KINECT METHODS            *
 *------------------------------------*/

/**
 * Also frees memory
 * @param s not used
 */
void cleanupExit(int s,kinectData &kd)
{
    kd.g_scriptNode.Release();
    kd.g_DepthGenerator.Release();
    kd.g_UserGenerator.Release();
    kd.g_Player.Release();
    kd.g_Context.Release();
    kd.recorder.Release();
    printf("All was cleaned up! Bye Bye!\n");
    exit (1);
}

/**
 * converts 3D point from projective coordinates to real world coordinates
 * @param  aProjective 3D point in projective coordinates
 * @return             3D point in real world coordinates
 */
XnPoint3D myConvertProjectiveToRealWorld(  XnPoint3D aProjective,kinectData &kd ){
    XnPoint3D aRealWorld = xnCreatePoint3D(.0, .0, .0);
    kd.g_DepthGenerator.ConvertProjectiveToRealWorld(1, &aProjective, &aRealWorld);
    
    return aRealWorld;
}

/**
 * converts 3D point from real world coordinates to projective coordinates
 * @param  aProjective 3D point in real world coordinates
 * @return             3D point in projective coordinates
 */
XnPoint3D myConvertRealWorldTorojective(  XnPoint3D aRealWorld ,kinectData &kd){
    XnPoint3D aProjective = xnCreatePoint3D(.0, .0, .0);
    kd.g_DepthGenerator.ConvertRealWorldToProjective(1, &aRealWorld, &aProjective);
    
    return aProjective;
}

int findAndPrepareNodesSimple(kinectData &kd){
    
    kd.nRetVal = kd.g_Context.Init();
    kd.nRetVal = kd.g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, kd.g_DepthGenerator);
    if (kd.nRetVal != XN_STATUS_OK)
    {
        //        printf("No depth generator found. Using a default one...");
        xn::MockDepthGenerator mockDepth;
        kd.nRetVal = mockDepth.Create(kd.g_Context);
        CHECK_RC(kd.nRetVal, "Create mock depth");
        
        // set some defaults
        XnMapOutputMode defaultMode;
        defaultMode.nXRes = 320;
        defaultMode.nYRes = 240;
        defaultMode.nFPS = 30;
        kd.nRetVal = mockDepth.SetMapOutputMode(defaultMode);
        CHECK_RC(kd.nRetVal, "set default mode");
        
        // set FOV
        XnFieldOfView fov;
        fov.fHFOV = 1.0225999419141749;
        fov.fVFOV = 0.79661567681716894;
        kd.nRetVal   = mockDepth.SetGeneralProperty(XN_PROP_FIELD_OF_VIEW, sizeof(fov), &fov);
        CHECK_RC(kd.nRetVal, "set FOV");
        
        XnUInt32 nDataSize = defaultMode.nXRes * defaultMode.nYRes * sizeof(XnDepthPixel);
        XnDepthPixel* pData = (XnDepthPixel*)xnOSCallocAligned(nDataSize, 1, XN_DEFAULT_MEM_ALIGN);
        
        kd.nRetVal = mockDepth.SetData(1, 0, nDataSize, pData);
        CHECK_RC(kd.nRetVal, "set empty depth map");
        
        kd.g_DepthGenerator = mockDepth;
    }
    
    
    return 0;
    
}

/**
 * function required by myConverXXToYY method.
 * @return
 */
int findAndPrepareNodes(kinectData &kd){
    
    //[ STAGE 2 ]
    // This node provides DEPTH information - depthMD
    kd.nRetVal = kd.g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, kd.g_DepthGenerator);
    if (kd.nRetVal != XN_STATUS_OK)
    {
        printf("No depth generator found. Using a default one...");
        xn::MockDepthGenerator mockDepth;
        kd.nRetVal = mockDepth.Create(kd.g_Context);
        CHECK_RC(kd.nRetVal, "Create mock depth");
        
        // set some defaults
        XnMapOutputMode defaultMode;
        defaultMode.nXRes = 320;
        defaultMode.nYRes = 240;
        defaultMode.nFPS = 30;
        kd.nRetVal = mockDepth.SetMapOutputMode(defaultMode);
        CHECK_RC(kd.nRetVal, "set default mode");
        
        // set FOV
        XnFieldOfView fov;
        fov.fHFOV = 1.0225999419141749;
        fov.fVFOV = 0.79661567681716894;
        kd.nRetVal   = mockDepth.SetGeneralProperty(XN_PROP_FIELD_OF_VIEW, sizeof(fov), &fov);
        CHECK_RC(kd.nRetVal, "set FOV");
        
        XnUInt32 nDataSize = defaultMode.nXRes * defaultMode.nYRes * sizeof(XnDepthPixel);
        XnDepthPixel* pData = (XnDepthPixel*)xnOSCallocAligned(nDataSize, 1, XN_DEFAULT_MEM_ALIGN);
        
        kd.nRetVal = mockDepth.SetData(1, 0, nDataSize, pData);
        CHECK_RC(kd.nRetVal, "set empty depth map");
        
        kd.g_DepthGenerator = mockDepth;
    }
    else{
        printf("[ok] Depth Node found\n");
    }
    // This node is used to be able access sceneMD
    kd.nRetVal = kd.g_Context.FindExistingNode(XN_NODE_TYPE_USER, kd.g_UserGenerator);
    if (kd.nRetVal != XN_STATUS_OK)
    {
        printf("[ok] Creating user generator...\n");
        // CHECK_RC(nRetVal, "Find user generator");
        kd.nRetVal = kd.g_UserGenerator.Create(kd.g_Context);
        CHECK_RC(kd.nRetVal, "Find Create user generator");
    }
    else
    {
        printf("[ok] User generator found\n");
    }
    
    
    // This node provides RGB information - imageMD
    kd.nRetVal = kd.g_Context.FindExistingNode(XN_NODE_TYPE_IMAGE, kd.g_ImageGenerator);
    if (kd.nRetVal != XN_STATUS_OK)
    {
        printf("Image node not found\n");
        // CHECK_RC(nRetVal, "Find image generator");
        kd.nRetVal = kd.g_ImageGenerator.Create(kd.g_Context);
        CHECK_RC(kd.nRetVal, "Find create image generator");
    }
    else
    {
        printf("[ok] Image node found\n");
    }
    
    //[ STAGE 3 ]       works but not from old onis: checked! From sensor or from recorded aligned oni
    XnBool isSupported = kd.g_DepthGenerator.IsCapabilitySupported("AlternativeViewPoint");
    if(TRUE == isSupported)
    {
        printf("ALIGNING\n");
        XnStatus res = kd.g_DepthGenerator.GetAlternativeViewPointCap().SetViewPoint(kd.g_ImageGenerator);
        if(XN_STATUS_OK != res)
        {
            printf("Getting and setting AlternativeViewPoint failed: %s\n", xnGetStatusString(res));
        }
    }else{
        //        printf("warning: ALIGNING NOT SUPPORTED\n");
    }
    
    return 0;
    
}

/*------------------------------------*
 *          PCL METHODS               *
 *------------------------------------*/



void pointPickingEvent (const pcl::visualization::PointPickingEvent &event,void*viewer_void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
    
    event.getPoint(coordx,coordy,coordz);
    printf("x = %f, y = %f, z = %f\n",coordx,coordy,coordz);
    //    sprintf (fname, "../../processing_data/txt/fr%d.txt", frameID);
    //    myfile.open ("../../processing_data/txt/frd.txt", std::ios_base::app);
//    ofstream myfile;
//    myfile.open (GROUNDTRUTH_PATH, std::ios_base::app);
//    myfile<<frameID << " " << coordx<< " " << coordy<< " " << coordz<<endl;
//    
//    myfile.close();
    //    viewer->resetCamera();
    
    //    viewer->close();
}

/**
 * PCL Visualizer mouse event handler
 * @param event
 * @param viewer_void
 */
void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
    if (event.getButton () == pcl::visualization::MouseEvent::RightButton &&
        event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
//        std::cout << "Right mouse button released at position (" << event.getX () << ", " << event.getY () << ")"  << std::endl;
        
        //viewer->resetCamera();
        
        viewer->close();
    }
}
/**
 * PCLVisualizer
 */
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, string message){
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    char str[400];
    sprintf(str,"%s Frame %d",message.data(),frameID);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer (str));
    viewer->setBackgroundColor (BCK_COLOR,BCK_COLOR,BCK_COLOR);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "sample cloud");
    //    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    //        viewer->setCameraClipDistances(3.1594,3159.4);
    //    viewer->setCameraPosition(-1136.52, -132.433, -694.066, -0.0520467, 0.998364, -0.0236548);
    
    //default: 562.575,3263.78/48.859,-39.8116,591.167/48.859,-39.8116,-1165.4/0,1,0/0.8575/640,400/50,123
    //from up 408.136,3458.19/48.859,-39.8116,591.167/20.9996,502.98,-1079.2/0.00410039,0.951059,0.308982/0.8575/640,400/50,123
    //    * clipping range (camera parameters)
    //    * focal point (camera parameters)
    //    * position (camera parameters)
    //    * view up (camera parameters)
    //    * view angle (camera parameters)
    //    * window size (window parameters)
    //    * window position (window parameters)
    
    
    //    viewer->setCameraPosition(15.6407,-829.263,-1890.46,0.00662797,0.948322,-0.317239);
    //    viewer->setCameraClipDistances(2.97301,2973.01);
    // viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
    viewer->setSize(640, 640);
    viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);
    viewer->registerPointPickingCallback(pointPickingEvent, (void*)&viewer);
    return (viewer);
}
/**
 * visualizes point cloud using PCLVisualizer
 * @param ref     pointcloud to visualize
 * @param message some message to display
 */
void visualizePoints(pcl::PointCloud<PointT>::Ptr &ref, string message){
    //    http://pointclouds.org/documentation/tutorials/pcl_visualizer.php
//    printf("visualizing\n");
    //    // ------------------------------VISUALIZE
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = rgbVis(ref,message);// rgbVis(pCloud);
    
    viewer->resetCamera();
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    
}


vector<pcl::PointCloud<PointT>::Ptr> extractClusters(pcl::PointCloud<PointT>::Ptr &cloud){
    //http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php
    //http://www.pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices
    //http://www.pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation
    //http://www.pcl-users.org/Cannot-adjust-parameters-for-Euclidean-Cluster-Extraction-to-work-td4027399.html
    pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<PointT> vg;
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    cloud_filtered=cloud;
    
    vector<pcl::PointCloud<PointT>::Ptr> retClusters;
    //    vg.setInputCloud (cloud);
    //    vg.setLeafSize (10.0000001f, 10.00000001f, 10.00000001f);
    //    vg.filter (*cloud_filtered);
    //    std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*
    //
    //    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
    
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIters);
    seg.setDistanceThreshold (distThrehold);
    
    int nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        
        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        
        // Write the planar inliers to disk
        extract.filter (*cloud_plane);
//        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
        
        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }
    
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud_filtered);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minClustSize);
    ec.setMaxClusterSize (maxClustSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);
    
    
    int j = 0;
    
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
            
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        // std::stringstream ss;
        // ss << "cloud_cluster_" << j << ".pcd";
//        visualizePoints(cloud_cluster,"cluster");
        // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        j++;
        retClusters.push_back(cloud_cluster);
    }
    return retClusters;
}

/**
 * downsamples pointcloud using voxel_grid filter of leafSize specified
 * @param  leafSize size of voxels
 * @return
 */
int filterWithVoxelG(float leafSize){
    /*Then, a pcl::VoxelGrid filter is created with a leaf size of 1cm(0.01f),
     the input data is passed, and the output is computed and stored in cloud_filtered.*/
    
    vxGrid.setInputCloud (pCloud);//
    vxGrid.setLeafSize (leafSize,leafSize,leafSize);
    vxGrid.setSaveLeafLayout (true);
    //    printf("go to filter\n");
    vxGrid.filter (*pCloudFiltered);
    
    //    printf("Filtering done.\n");
    int sizeVxGrid = pCloudFiltered->width * pCloudFiltered->height ;
    
    // cout << "PointCloud after filtering: " << sizeVxGrid << " data points (" << pcl::getFieldsList (*pCloudFiltered) << ").";
    
    return sizeVxGrid;
}

/**
 * sets RGB color to PointXYZRGB
 * @param p point to be colored
 * @param r red color
 * @param g green color
 * @param b blue color
 */
void setColor(PointT &p, int r, int g, int b){
    p.r = r;
    p.g = g;
    p.b = b;
}

/**
 * constructs graph based on pointcloud
 * @param  nodes number of nodes in graph
 * @return graph structure (vector of vectors)
 */
vector < vector< pairInts > > buildGraph(int nodes){
    vector < vector< pairInts > > graph(nodes) ;
    int edges = 0;
    int  u=0, v=0, w=0;
    
    for (int i = 0; i < nodes; ++i)
    {
        Indist ret = getNeighborIndices(i);
        // printf("preproceso: %d tiene %d vecinos\n", i, (int)ret.neighborIndices.size());
        for (int j = 0; j < ret.neighborIndices.size(); ++j)//add each neighbour
        {
            edges++;
            u = i; v = ret.neighborIndices[j]; w=ret.distances[j];
            graph[u].append(pairInts(v,w));//G[A] tiene arista hacia B con distancia w
            // G[v].append(pairInts(u,w)); // for undirected y numNeigbor sera doble
        }
    }
    return graph;
}

/**
 * Searches all short paths from initial node to all others in the graph
 * @param starting starting node from where begin
 * @param nodes    number of nodes in graph
 * @param graph    graph with nodes
 */
int dijkstra(int starting, int nodes, vector < vector< pairInts > > graph ){
    priority_queue< pairInts, vector < pairInts >, comp > pri_queue;
    
    vector<int> dist(nodes);
    vector<int> normalized_D(nodes);
    vector<bool> visit(nodes);
    vector<int> prev(nodes);
    // initialize dijkstra
    // printf("dijkstra:initializing...\n");
    for (int u = 0; u < nodes; ++u){
        dist[u] = INF;
        prev[u] = -1;
    }
    int  u, v, w, numNeighbor;
    int maxD = -1;
    int minD = MAXIMUM;
    
    dist[starting] = 0;
    
    vector< int > geoval;
    
    pri_queue.push(pairInts(starting, 0));                  // pongo primero en la cola
    
    //dijkstra
    // printf("dijkstra:  while...\n");
    while(!pri_queue.empty())
    {
        u = pri_queue.top().first;//get min
        if(u < 0 && u >= visit.size()){
            //            printf("u=%d\n",u);
            break;
        }
        // printf("Starting with node %d\n", u);
        pri_queue.pop(); //delete it from pri_queue
        //        printf("Trying to visit[%d]\n",u);
        
        if(visit[u]) continue;    //si ya lo habiamos visitado, pasamos a siguiente iteracion
        
        numNeighbor = graph[u].size();
        // printf("%d has %d neighbours\n", u, numNeighbor);
        // printf("visiting from %d:", u );
        
        for (int i = 0; i < numNeighbor; ++i)
        {
            //cojo el B que conecta con graph[A]
            v = graph[u][i].first;                      //cojo al primer vecino
            //y la distancia que los separa
            w = graph[u][i].second;                     //y distancia hacia el
            
            if(!visit[v] && dist[u] + w < dist[v]){
                //printf(" %d, ", v);
                dist[v] = dist[u] + w;
                if(dist[v] > maxD){
                    maxD = dist[v];
                }else if(dist[v] < minD) {
                    minD = dist[v];
                }
                prev[v] = u; // <-- added line   //padre del vecino era u
                //printf("prev[%d]=%d\n",v,prev[v] );
                pri_queue.push(pairInts(v,dist[v]));
            }
        }//end for
        //printf("\n");
        visit[u] = 1; // done with u
    }
    // printf("dijkstra:fin while\n");
    //put all dists in range 0-255
    
    
    
    //    printf("dijkstra:gonna put color on nodes\n");
    //recorro todas las distancias geodesicas
    bool used= false;
    for (int i = 0; i < nodes; ++i)                //WARNING #3 : orden
    {
        if(dist[i] == INF){
            // printf("INIFINITOOOO!\n");
            
            setColor(pCloudFiltered->points[i],0,255,0); // WARNING #1
            geoval.push_back(0); // WARNING #2 : implica sort!  WARNING #3 : orden
            
        }else{
            
            geoval.push_back(dist[i]);  //WARNING #3 : orden
            
            normalized_D[i] = ((dist[i] - minD)*(255.f/((float)maxD-(float)minD)));
            int red = 255-normalized_D[i];
            int blue = normalized_D[i];
            //            if(dist[i]>600){
            //            printf("high blue point: - dist[%d]=%d\n",i,dist[i]);
            setColor(pCloudFiltered->points[i],red,0,blue);
            //            }
            
        }
        
        // //printf("normalized_D:%d\n",normalized_D[i] );
    }
    
    
    // UNCOMENT TO WRITE HDD
    
    ofstream myfile;
    //    myfile.open (GEOVALS_PATH);
    //    cout << "dist.size: "<< dist.size() <<endl;
    //    cout << "geoval.size: "<<geoval.size()<<endl;
    //    sort (geoval.begin(), geoval.end()); //WARNING #3 : orden
    int maxGeoVal = 0;
    for (int i = 0; i < geoval.size(); ++i)
    {
        if(geoval[i] > maxGeoVal) maxGeoVal=geoval[i];
        //        myfile << geoval[i] << endl; //WARNING #3 : orden
    }
    
    glb_geoval = geoval;
    
    //    myfile.close();
    //    printf("geoval has been written to file\n");
    
    //    printf("dijkstra:returning maxGeoVal = %d\n",maxGeoVal);
    
    return maxGeoVal;
}
// --------------------------------------------------------------------------------------------------------------


float meanFlow(int radius,const Mat& flow,int x, int y){
    float meanF = 0.0;
    float cont = 0.0;
    
    if((y-radius)>0 && (x-radius)>0 && (y+radius)<yRes && (x+radius)<xRes  ){ // checking boundaries
        
        for(int yy = y - radius; yy < y + radius; yy++ ){
            
            for(int xx = x - radius; xx < x+ radius ; xx++){
                
                const Point2f& fxy = flow.at<Point2f>(yy,xx);
                if(fxy.x != 0.0 && fxy.y != 0.0){ //TODO: fix if glb_flow null. peta
                    //                                       cout << "fx="<<fxy.x << endl;
                    //                                        cout << "fy="<<fxy.y << endl;
                    
                    meanF += sqrt(pow(fxy.x,2) + pow(fxy.y,2) );
                    
                    cont++;
                }
                
            }
        }
    }
    //cout << "meanF:"<<meanF<<" cont:" << cont<< " res="<<meanF/cont <<endl;
    return meanF/cont;
}

float eucDistance (XnPoint3D p1, XnPoint3D p2)
{
    float diff_x = p2.X - p1.X, diff_y = p2.Y - p1.Y, diff_z = p2.Z - p1.Z;
    return (diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);
}

void printPointCoordinates(XnPoint3D &p, char *msg){
    printf("%s ",msg);
    printf("point coordinates are: x=%f,y=%f,z=%f\n",p.X,p.Y,p.Z);
}
void printDifToFile(XnPoint3D input, XnPoint3D out,char *glb_filename){
    float difx, dify,difz;
    printf("geoval coordinates are: x=%f,y=%f,z=%f\n",input.X,input.Y,input.Z);
    printf("gtruth coordinates are: x=%f,y=%f,z=%f\n",out.X,out.Y,out.Z);
    difx = abs(input.X - out.X);
    dify = abs(input.Y - out.Y);
    difz = abs(input.Z - out.Z);
    printf("dif coordinates are: x=%f,y=%f,z=%f\n",difx,dify,difz);
    ofstream myfile;
    int detected = -1;
    
    if (difx < threshold_mm && dify < threshold_mm) {
        detected = 1;
    }else{
        detected = 0;
    }
    if(out.X != 0 && out.Y != 0&& out.Z != 0){
    myfile.open (glb_filename, std::ios_base::app);
    myfile<<frameID << " " << difx<< " " << dify<< " " << difz<< " "<< detected << endl;
    
    myfile.close();
    }else{
        cout << "Skipping frame as gtrh not found" << endl;
    }
}

int readGroundTruth(vector<pair<XnPoint3D,XnPoint3D> > &groundtruthPoints,char *name){
    pair<XnPoint3D,XnPoint3D> twoPoints;
    XnPoint3D p, p2;
    p.X = 0, p.Y = 0, p.Z = 0;  //TODO: assuming that 0,0,0 is invalid coordinate
    p2.X = 0, p2.Y = 0, p2.Z = 0;
    twoPoints.first = p, twoPoints.second = p2;
    int frame;
    std::string line;
    char filename[100];
    sprintf(filename,"%s%s_coord.txt",DB_PATH,name);
    std::ifstream backstory(filename);
    bool stop = false;
    
	if (backstory.is_open())
	{
		while (backstory.good() || stop == true)
		{
            getline(backstory,line);
            std::vector<std::string> strs;
            boost::split(strs, line, boost::is_any_of(" "));
            if(strs.size()<3){
                stop=true;
                break;
            }
            frame = atoi(strs.at(0).c_str());
            p.X=atof(strs.at(1).c_str());
            p.Y=atof(strs.at(2).c_str());
            p.Z=atof(strs.at(3).c_str());
            twoPoints.first = p;
            line.clear(),strs.clear();
            getline(backstory,line);
            boost::split(strs, line, boost::is_any_of(" "));
            if(strs.size()<3)stop=true;
            int newFrame = atoi(strs.at(0).c_str());
            if(frame == newFrame){//TODO: if three same frames found, last will be taken
                p2.X=atof(strs.at(1).c_str());
                p2.Y=atof(strs.at(2).c_str());
                p2.Z=atof(strs.at(3).c_str());
                twoPoints.second = p2;
            }else{
                twoPoints.second.X = 0, twoPoints.second.Y = 0, twoPoints.second.Z = 0;
            }
            groundtruthPoints.at(frame) =  twoPoints; // each point goes to its corresponding frame pos
            
            
        }//while
        backstory.close();
    }else{
        std::cout << "Unable to open file" << std::endl << std::endl;
        return -1;
    }
    return 0;
    
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/**
 * Draws lines and points on image from optical flow map
 * @param flow     resulting image matrix
 * @param cflowmap map of optical flow vectors
 * @param step     separation between pixels to calculate optical flow
 * @param scale    not used
 * @param color    color of vectors (green)
 * @param maxFlowXY  adress of vector with coordinates
 * @param th_maxflow threshold of flow vector magnitude
 */


void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step, double scale, const Scalar& color, vector<pairInts> &maxFlowXY,float th_maxflow){
    
    //recorrido por todos pixeles con separacion 'step'
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {
            
            
            
            const Point2f& fxy = flow.at<Point2f>(y,x);
            //                        printf("at(%d,%d)->fx:%f fy:%f\n",x,y,fxy.x,fxy.y);
            if(abs(fxy.x) > th_maxflow || abs(fxy.y) > th_maxflow){
                maxFlowXY.push_back(pairInts(x,y) );
            }
            line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                 color);
            circle(cflowmap, Point(x,y), 1, color, -1);
            // circle(cv::Mat &img, Point center, int radius, const Scalar &color)
            
            
            
        }
    
    //pinto solo los que son de interes ( mucho movimiento)
    for (int i = 0; i < maxFlowXY.size(); ++i)
    {
        //printf("max flow X:%d,Y:%d\n",maxFlowXY[i].first,maxFlowXY[i].second );
        circle(cflowmap, Point(maxFlowXY[i].first,maxFlowXY[i].second), 3, CV_RGB(0, 121, 255)); // COLOR
    }
    
    
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/**
 * Determine optical flow by Farneback algorithm based on
 * dense optical flow idea ( for each pixel )
 * @param frame1 first frame in Mat format
 * @param frame2 second frame in Mat format
 * @param th_maxFlow threshold of flow vector magnitude
 * @param saveImg save image of optical flow to disk or not
 * @return vector<pairInts> with coordinates of points to remove
 */
vector<pairInts> opt_flow(Mat frame1, Mat frame2,float th_maxflow,bool saveImg){
    vector<pairInts> pxToRemove = vector<pairInts>(); //empty dynamic vector
    // IplImage *frame1 = cvLoadImage("../../1.png");
    // IplImage *frame2 = cvLoadImage("../../2.png");
    
    // Mat frame1 = imread("../../1.png", CV_LOAD_IMAGE_COLOR);
    // Mat frame2 = imread("../../2.png", CV_LOAD_IMAGE_COLOR);
    
    
    Mat prevgray, gray, flow, cflow, frame;
    //    namedWindow("flow", 1);
    
    
    if(frame1.step.buf[1] == 3){            //convert to gray only if it's RGB
        cvtColor(frame1, prevgray, CV_BGR2GRAY);
    }else{
        prevgray = frame1;                      //leave it as it is
    }
    if(frame2.step.buf[1] == 3){            //convert to gray only if it's RGB
        cvtColor(frame2, gray, CV_BGR2GRAY);
    }else{
        gray = frame2;                      //leave it as it is
    }
    
    //    // imshow("1",frame1);waitKey(0);imshow("2",frame2);waitKey(0);        //IMSHOW1
    
    
    /*
     InputArray prev
     InputArray next
     InputOutputArray flow
     double pyr_scale
     int levels
     int winsize     : como de precisos son las inclinaciones de los vectores
     int iterations
     int poly_n
     double poly_sigma
     int flags
     */
    
    calcOpticalFlowFarneback(prevgray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
    
    
    glb_flow = flow;
    
    //    return pxToRemove;
    //    flow.create(prev.size(), CV_32FC2);
    //    flow = Scalar(0);
    //    Point origin((flow.cols - flow_received.cols)/2, (flow.rows - flow_received.rows)/2);
    //    Mat wcp = flow(Rect(origin, flow_received.size()));
    //    flow_received.copyTo(wcp);
    
    cvtColor(prevgray, cflow, CV_GRAY2BGR);
    
    drawOptFlowMap(flow, cflow, GRANULARITY, -1, CV_RGB(0, 255, 0),pxToRemove,th_maxflow);
    
//         imshow("flow", cflow);  waitKey(0);      //IMSHOW1
    if(saveImg){
        //        char filename[512];
        //        sprintf (filename, OPTFLOW_PATH, frameID);
        //        printf("saving optflow to disk\n");
        //        imwrite(filename, cflow);
    }else{
        printf("optflow not saved to disk\n");
    }
    
    return pxToRemove;
    
    
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/**
 * helper function to call optical flow method and remove nodes which has big movement
 * @param  th_moveAround threshold to remove nodes in that radius
 * @param  th_maxflow    threshold of flow vector magnitude
 * @param  saveImg       save image of optical flow to disk or not
 * @return
 */
vector<pairInts> removeHighFlow(int th_moveAround,float th_maxflow,bool saveImg){
    
    vector<pairInts> pointsToRemove = opt_flow(imageA,imageB,th_maxflow,saveImg);
    
    int color = 0;
    int th_nulate = 0;
    //    cout<<"removeHighFlow: th_maxflow = "<<th_maxflow<<endl;
    // visualizePoints(pCloud, "hello");
    for(int ka = 0; ka < pointsToRemove.size(); ka++){
        int x = pointsToRemove[ka].first;
        int y = pointsToRemove[ka].second;
        // printf("---------x:%d,y:%d\n",x,y );
        
        // printf("VVA:x:%f,y:%f\n",pCloud->at(x,y).x,pCloud->at(x,y).y);
        for(int vert = -th_moveAround; vert < th_moveAround; ++vert){
            for(int horiz = -th_moveAround; horiz < th_moveAround; ++horiz){
                int newX = x+horiz;
                int newY = y+vert;
                if(newX < 0) newX = 0;//TODO : workaround
                if(newY < 0) newY = 0;
                if(newX >= 640) newX = 639;
                if(newY >= 480) newY = 479;
                if(pCloud->at(newX,newY).z != 0){
                    setColor(pCloud->at(newX,newY), color,150,color); // WARNING #1 no hace nada, la distancia en el dijkstra sera INF ya que anulo z
                    pCloud->at(newX,newY).z = th_nulate; //ONLY Z!
                }
            }
        }
    }
    //visualizePoints(pCloud, "after");
    return pointsToRemove;
}
/**
 * builds Mat with resulting final image of geoflow map. Processes resulting pointcloud to get pixels of certain color
 * saves image to disk if required and returns Mat
 * @param  target cloud reference
 * @param  saveToDisk whether save or not to disk
 * @return            Mat with resulting image of geoflow
 */
Mat saveResult(pcl::PointCloud<PointT>::Ptr &cloud, bool saveToDisk,kinectData &kd){
    Mat result = Mat::zeros(yRes,xRes, CV_8UC3);
    int nodes = cloud->size();
    
    //    printf("saveResult::total points:%d\n",nodes);
    for (int i = nodes; i > 0; --i)
    {
        PointXYZRGB punt = cloud->points[i];
        XnPoint3D pp2 = xnCreatePoint3D(punt.x, punt.y, punt.z);
        XnPoint3D prw2 = myConvertRealWorldTorojective(pp2,kd);
        
        if(prw2.X >0 && prw2.Y > 0 && prw2.X < xRes && prw2.Y < yRes ){
            if(punt.g != 255){ // WARNING #1 !
                // int b = int(punt.b);
                // int g = int(punt.g);
                // int r = int(punt.r);
                int y = int(prw2.Y);
                int x = int(prw2.X);
                
                // result.at<cv::Vec3b>(y,x)[0] = b;
                // result.at<cv::Vec3b>(y,x)[1] = g;
                // result.at<cv::Vec3b>(y,x)[2] = r;
                result.at<cv::Vec3b>(y,x)[0] = 255;
                result.at<cv::Vec3b>(y,x)[1] = 255;
                result.at<cv::Vec3b>(y,x)[2] = 255;
                
                
            }
        }
        
    }
    if (saveToDisk){
        //        char filename[512];
        //        sprintf (filename, "../../processing_data/images/other_result%d.png", frameID);
        
        //        imwrite(filename, result);
    }
    //    //    imshow("Final result",result); waitKey(0);       //IMSHOW1
    
    
    
    return result;
}

void cut_iVector(vector<pairInts> &vec,float percentage){
    //cutting out vector to adjust percentage
    int gsz = vec.size();
//    cout<<"gsz="<<gsz<<endl;
    int target_gsz = gsz * percentage;
    while(vec.size() > target_gsz)vec.pop_back();
//    cout<<"new gsz="<<vec.size()<<endl;
    
}

int getCentroid(Eigen::Vector4f &centroid, pcl::PointCloud<PointT>::Ptr &cloud){
    
    compute3DCentroid(*cloud,centroid);
    //http://docs.pointclouds.org/trunk/group__common.html#ga26f5d53ac5362b04a5c8ed68c4c39038
    //    cout << "centroid: "<<centroid[0] << " "<<centroid[1] << " "<<centroid[2] << " "<<endl;
    
    //    visualizePoints(cloud, "clusters_clouds");
    return 0;
}


bool mycomparator ( const pairInts& l, const pairInts& r)
{ return l.first < r.first; }

bool mycomparatorFinalists ( const valPoint &v1,const valPoint &v2)//should be const!!!
{ return v1.v < v2.v; }
MatVec post_process(pcl::PointCloud<PointT>::Ptr &cloud, bool saveToDisk,bool removeHighFlow,int maxGeoVal,vector<pairInts> oldHands, char * glb_filename,kinectData &kd,char *name){
    PointCloud<PointT>::Ptr pCloudGeodesic (new PointCloud<PointT>);
    
    Mat result = Mat::zeros(yRes,xRes, CV_8UC3);
    result.setTo(cv::Scalar(255,255,255));
    vector<float> retVec;
    MatVec ret;
    int pos= 0;
    int nodes = cloud->size();
    //initialize vector of floats to 0
    //el size de este es el ultimo valor geodesico ya que esta ordenado
    //    vector<float> histogram(glb_geoval.at(glb_geoval.size()-1)+1,0); // TO CHECK!
    vector<float> histogram(maxGeoVal,0);
    vector<int> conters(histogram.size(),0);
    vector<XnPoint3D > points;
    //    printf("histogram.size=%lu,conters.size=%lu\n",histogram.size(),conters.size());
    //    printf("post_process::total points:%d\n",nodes);
    //drawing a track
    //TODO: make it with points
    
    // sorting geoval vector keeping track of indexes
    //    http://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
    
    vector<pair<XnPoint3D,XnPoint3D> > groundtruthPoints(10000);//TODO: to adjust num of points total
    readGroundTruth(groundtruthPoints,name);
    
    
    vector<pairInts> geovals;
    for (int k=0; k<glb_geoval.size(); k++) {
        pairInts tmp = pairInts(glb_geoval.at(k),k);
        geovals.push_back(tmp);
    }
    sort (geovals.begin(), geovals.end(),mycomparator);//ascending
    reverse(geovals.begin(),geovals.end());
    cut_iVector(geovals,threshold_numpoints);
    
    for(int m = 0; m<geovals.size(); m++){
        //        if(glb_geoval[d] >= geovals[d].first){// >= maxGeoVal-30
        int d = geovals[m].second;//getting the index
        setColor(cloud->points[d],0, 0, 0);  //GREEN TO MARK DETECTED HAND 10,250,10
        
        //printf("highest geoval at: d=%d\n",d);
        float inx,iny,inz;
        
        inx = cloud->points[d].x;
        iny = cloud->points[d].y;
        inz = cloud->points[d].z;
        //        printf("%d - geodes coordinates are: x=%f,y=%f,z=%f\n",frameID,inx,iny,inz);
        XnPoint3D pu = {inx,iny,inz} ;
        
        points.push_back(pu);
        
        pCloudGeodesic->push_back(cloud->points[d]);
        
        
        //        }//if
        
        
        
        
    }//for
    
//        visualizePoints(cloud, "cloud check");
//    
//        visualizePoints(pCloudGeodesic, "pCloudGeodesic ");
    
    vector<PointCloud<PointT>::Ptr> clusters_clouds;
    
    clusters_clouds = extractClusters(pCloudGeodesic);
    
    int csz = clusters_clouds.size();
    if(csz > 0){
        cout<<"clusters_clouds size = "<<csz<<endl;
        Eigen::Vector4f centroid;
        // randomly pick only two clouds (at max we will have two hands)

        XnPoint3D bestPoint;
        XnPoint3D bestPoint2;//for 2 hand case
        vector<XnPoint3D> best_points;
        if(csz == 1){ //one geodesic hand case
            
            getCentroid(centroid,clusters_clouds.at(0));
            XnPoint3D p = {centroid[0],centroid[1],centroid[2]};
            
            float a = eucDistance(p, groundtruthPoints.at(frameID).first);
            float b = eucDistance(p, groundtruthPoints.at(frameID).second);
            
            if(a < b)bestPoint = groundtruthPoints.at(frameID).first;
            else bestPoint = groundtruthPoints.at(frameID).second;
            printDifToFile(p,bestPoint,glb_filename);
            
        }else{//two geodesic hands case
            XnPoint3D hand1, hand2;
            int i = 0;
            getCentroid(centroid,clusters_clouds.at(i));
            hand1.X = centroid[0],hand1.Y = centroid[1],hand1.Z = centroid[2];
            i++;
            getCentroid(centroid,clusters_clouds.at(i));
            hand2.X = centroid[0],hand2.Y = centroid[1],hand2.Z = centroid[2];
            i++;
            while(i<clusters_clouds.size()){// discarting head by vertical position
                getCentroid(centroid,clusters_clouds.at(i));
                if(centroid[1]<hand1.Y ){
                    cout<<"caso!!!!!!!!!!!!!!!!1"<<endl;
                    hand1.Y = centroid[1];
                }else if (centroid[1] < hand2.Y){
                    hand2.Y = centroid[1];
                    cout<<"caso!!!!!!!!!!!!!!!!2"<<endl;
                }
                i++;
            }
            //        while(cont>0){
            //
            //            int output = 0 + (rand() % (int)(clusters_clouds.size()-1 - 0 + 1));
            //            if(usedVal != output){
            //                getCentroid(centroid,clusters_clouds.at(output));
            //                cout<<"used index:"<<output<<endl;
            //                cont--;
            //                if(output==1)hand1.X = centroid[0],hand1.Y = centroid[1],hand1.Z = centroid[2];
            //                else hand2.X = centroid[0],hand2.Y = centroid[1],hand2.Z = centroid[2];
            //            }
            //            usedVal = output;
            //        }//while
            
            //TODO: remake this stupid with pairing (a,point) and ranking two best
            printPointCoordinates(hand1,"hand1");
            printPointCoordinates(hand2,"hand2");
            printPointCoordinates(groundtruthPoints.at(frameID).first,"ghand1");
            printPointCoordinates(groundtruthPoints.at(frameID).second,"ghand2");
            float a = eucDistance(hand1, groundtruthPoints.at(frameID).first);
            float b = eucDistance(hand2, groundtruthPoints.at(frameID).second);
            float c = eucDistance(hand2, groundtruthPoints.at(frameID).first);
            float d = eucDistance(hand1, groundtruthPoints.at(frameID).second);
            vector<valPoint>  finalist;
            valPoint temp;
            temp.v = a;
            pair<XnPoint3D,XnPoint3D> parP(hand1,groundtruthPoints.at(frameID).first);
            temp.par = parP;
            finalist.push_back(temp);
            
            temp.v = b;
            parP.first = hand2;
            parP.second = groundtruthPoints.at(frameID).second;
            temp.par = parP;
            finalist.push_back(temp);
            
            
            temp.v = c;
            parP.first = hand2;
            parP.second = groundtruthPoints.at(frameID).first;
            temp.par = parP;
            finalist.push_back(temp);
            
            temp.v = d;
            parP.first = hand1;
            parP.second = groundtruthPoints.at(frameID).second;
            temp.par = parP;
            finalist.push_back(temp);
            
            
            sort(finalist.begin(),finalist.end(),mycomparatorFinalists);
            reverse(finalist.begin(),finalist.end());
            
            bestPoint = finalist.back().par.first;//geoval
            bestPoint2 = finalist.back().par.second;//gtruth
            finalist.pop_back();
            
            printDifToFile(bestPoint, bestPoint2,glb_filename);
            
            bestPoint = finalist.back().par.first;
            bestPoint2 = finalist.back().par.second;
            finalist.pop_back();
            printDifToFile(bestPoint, bestPoint2,glb_filename);
            
        }//else
        
    }//if
    
    //    visualizePoints(pCloudFiltered, "check");
    //recorro los puntos conexos por geoflow
    //        visualizePoints(cloud, "saveR2");
    for (int i = nodes; i > 0; --i)          //WARNING #3 : orden
    {
        PointXYZRGB punt = cloud->points[i];
        XnPoint3D pp2 = xnCreatePoint3D(punt.x, punt.y, punt.z);
        XnPoint3D prw2 = myConvertRealWorldTorojective(pp2,kd);
        
        if(prw2.X >0 && prw2.Y > 0 && prw2.X < xRes && prw2.Y < yRes ){
            
            if(punt.g != 255){ // WARNING #1 !
                int b = int(punt.b);
                int g = int(punt.g);
                int r = int(punt.r);
                int y = int(prw2.Y);
                int x = int(prw2.X);
                
                //connecting previous hands
                if(punt.r == 10 && punt.g == 250){
                    ret.hands.push_back(pairInts(x,y));
                }
                
                //construyo la imagen resultante en bucles de radio para quitar espacio entre puntos
                int rad = 10;
                for(int vert = -rad; vert < rad; ++vert){
                    for(int horiz = -rad; horiz < rad; ++horiz){
                        int newX = x+horiz;
                        int newY = y+vert;
                        if(newX < 0) newX = 0;//TODO : workaround
                        if(newY < 0) newY = 0;
                        if(newX >= 640) newX = 639;
                        if(newY >= 480) newY = 479;
                        result.at<cv::Vec3b>(newY,newX)[0] = b;
                        result.at<cv::Vec3b>(newY,newX)[1] = g;
                        result.at<cv::Vec3b>(newY,newX)[2] = r;
                    }
                }
                if(removeHighFlow){
                    //para cada punto calculo el optflow promedio en vecindad de un radio en coord x,y
                    float mf = meanFlow(3, glb_flow, x,y);
                    //printf("at(%d,%d)->  mf=%f\n",x,y,mf);
                    
                    //voy colocando en posicion con valor geodesico correspondiente
                    if(i<glb_geoval.size()){
                        //indexo con pos el valor geodesico correspondiente al punto de cloud actual
                        pos =glb_geoval.at(i);//WARNING #3 : orden
                        
                        //                        printf("pos: %d\n",pos);
                        //mientras no he llenado todo vector
                        if(pos >= 0 && pos < histogram.size()-1){
                            //voy sumando en caso que varios puntos tengan mismo valor geodesico
                            
                            //[trick]: ya esta ordenado. Un vector va 0,1,2,3,4,5,6,7,8,9,...
                            //solo falta indexar con pos y sumar mean flow en posicion determinada
                            histogram.at(pos) += mf;
                            conters.at(pos)++;
                            
                            //muestro en imagen solo si mean flow alto
                            if(mf>9){
                                //para poner texto en imagen
                                char* str = new char[30];
                                sprintf(str, "%5.1f", mf );
                                //posiciones X pares
                                if(x%2==0){
                                    // cout << "mF mean flow: "<<str <<endl;
                                    //putText(result, str, cvPoint(x,y),FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(200,90,250),1);
                                }
                            }
                        }
                    }
                    
                }
            }else{
                
                //cout << "->post_process: green detected : 255" << endl;
            }
            
        }
        
    }
    
    for (int k = 0; k<oldHands.size(); k++) {
        ret.hands.push_back(oldHands.at(k));
    }
    for (int k = 0; k<oldHands.size(); k++) {
        result.at<cv::Vec3b>(oldHands.at(k).second,oldHands.at(k).first)[0] = 10;//b
        result.at<cv::Vec3b>(oldHands.at(k).second,oldHands.at(k).first)[1] = 250;//g
        result.at<cv::Vec3b>(oldHands.at(k).second,oldHands.at(k).first)[2] = 10;//r
    }
    
    
    //    int dilation_elem = 0;
    //    int dilation_size = 4;
    //    int dilation_type = -1;
    //    if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
    //    else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
    //    else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
    //
    //    Mat element = getStructuringElement( dilation_type,
    //                                        Size( 2*dilation_size + 1, 2*dilation_size+1 ),
    //                                        Point( dilation_size, dilation_size ) );
    //
    //    dilate( result, result, element );
    //    imshow("dilate",result);
    //    waitKey(0);
    
    if (saveToDisk){
        char filename[512];
        //        sprintf (filename, "../../processing_data/images/result%d.png", frameID);
//        sprintf (filename, RESULT_PATH, frameID);
//        printf("saving to disk %s\n",filename );
//        imwrite(filename, result);
    }else{
        printf("not saved to disk\n");
    }
//            imshow("Final result",result); waitKey(0);      //IMSHOW1
    
    // UNCOMENT TO WRITE HDD
    
    if(removeHighFlow){
//        char fname[512];
        
//        sprintf (fname, MEAN_VEC_PATH, frameID);
        //        sprintf (fname, "../../processing_data/txt/meanVec-%d.txt", frameID);
        //        ofstream myfile;
        //        myfile.open (fname);
        //TODO: por lo que parece falta ultimo valor, no llega todo recorrido
        for(int k=0;k<conters.size();k++){
            if(conters.at(k)!=0 && histogram.at(k)!=0){
                //                printf("writing to file vec1(%d): %f/%d=%f\n",k,histogram.at(k),conters.at(k),histogram.at(k)/conters.at(k));
                float val = histogram.at(k)/conters.at(k);
                //                myfile << val << endl;
                retVec.push_back(val);
            }else{
                //                myfile << 0<<endl;
                //printf("writing to file: %f\n",0.0);
                retVec.push_back(0);
            }
        }
        //        myfile.close();
        
        ret.vec = retVec;
    }
    ret.matr = result;
    return ret;
}

/**
 * helper function to call filterWithVoxelG
 * @param  leafSize size of voxels
 * @return
 */
int down_sample(float leafSize)
{
    //    printf("vox grid:\n");
    int nodes, numPoints;
    numPoints = pCloud->width * pCloud->height;
    nodes = filterWithVoxelG(leafSize);
    //    printf("Size of pCloudFiltered:%d\n",nodes );
    //    printf("voxelize:returning \n");
    return (nodes);
}

pairInts findCentre(Mat inputIm){
    
    pairInts centIndx = pairInts(0,0);
    
    int maxVal,maxX,maxY;
    maxVal=maxX=maxY=0;
    
    //    imshow("inputIm",inputIm); waitKey(0);     //IMSHOW1
    Mat fin=Mat::zeros(480,640, CV_8UC1);
    cv::inRange(inputIm, cv::Scalar(85), cv::Scalar(102), fin);
    // threshold(inputIm, inputIm, 85, 102, CV_THRESH_BINARY);
    Canny( fin, fin, 49, 147, 3 );
    
    //    // imshow("Canny",fin);//IMSHOW1
    
    Mat cannyInv=Mat::zeros(480,640, CV_8UC1);
    bitwise_not(fin, cannyInv);
    //    // imshow("cannyInv",cannyInv);//IMSHOW1
    
    int dilation_elem = 0;
    int dilation_size = 4;
    int dilation_type = -1;
    if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
    else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
    else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
    
    Mat element = getStructuringElement( dilation_type,
                                        Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                        Point( dilation_size, dilation_size ) );
    Mat drawing2=Mat::zeros(480,640, CV_8UC1);;
    dilate( fin, drawing2, element );
    vector< vector<Point> > contours;
    findContours(drawing2, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    Mat mask = drawing2;
    // CV_FILLED fills the connected components found
    drawContours(mask, contours, -1, Scalar(255), CV_FILLED);
    
    Mat dist=Mat::zeros(480,640, CV_8UC1);;
    distanceTransform( cannyInv, dist, CV_DIST_L2, 3 );
    Mat distNorm;
    
    dist.convertTo(distNorm, CV_8UC1,1,0);
    Mat result= Mat::zeros(480,640, CV_8UC1);
    distNorm.copyTo(result, mask);
    Mat tmp=Mat::zeros(480,640, CV_8UC1);
    Mat fik=Mat::zeros(480,640, CV_8UC3);
    
    //    imwrite(DISTMAP_PATH,result);
    for(int i = 0; i < result.rows; i++)
    {
        for(int j = 0; j < result.cols; j++)
        {
            if ( result.at< uchar >( i,j ) > 0){
                uchar val = result.at< uchar >( i,j  );
                if(val>maxVal){
                    if(val>0){
                        cv::circle(tmp,cvPoint(j,i),2,255,-1);
                    }
                    maxVal=val;
                    maxX = j;
                    maxY = i;
                }
            }
        }
    }
    cv::circle(result,cvPoint(maxX,maxY),5,255,-1);
    Mat planes[] = {result, result, tmp};
    merge(planes, 3, fik);
    
    //    printf( "x: %i y:%i  val: %d\n",maxX,maxY, maxVal );
    //    imwrite(CENTR_PATH,fik);
    result.release();
    dist.release();
    distNorm.release();
    mask.release();
    cannyInv.release();
    fin.release();
    inputIm.release();
    
    centIndx.first = maxX;
    centIndx.second = maxY;
    return centIndx;
}
pairInts findCentreCompl(Mat inputIm){
    
    pairInts centIndx = pairInts(0,0);
    
    int maxVal,maxX,maxY;
    maxVal=maxX=maxY=0;
    
    // Only work with grayscale images
    cv::cvtColor(inputIm, inputIm, CV_RGB2GRAY);
    
    //    // imshow("inputIm",inputIm);//IMSHOW1
    
    int dilation_elem = 0;
    int dilation_size = 4;
    int dilation_type = -1;
    if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
    else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
    else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
    
    Mat element = getStructuringElement( dilation_type,
                                        Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                        Point( dilation_size, dilation_size ) );
    Mat drawing2=Mat::zeros(480,640, CV_8UC1);;
    dilate( inputIm, inputIm, element );
    //    // imshow("dilate",inputIm);//IMSHOW1
    
    Mat fin=Mat::zeros(480,640, CV_8UC1);
    // cv::inRange(inputIm, cv::Scalar(3), cv::Scalar(102), fin);
    // threshold(inputIm, inputIm, 85, 102, CV_THRESH_BINARY);
    blur( inputIm, inputIm, Size(3,3) );
    
    Canny( inputIm, fin, 500, 1000, 3 );
    
    //    // imshow("Canny",fin);//IMSHOW1
    
    
    
    
    
    
    Mat cannyInv=Mat::zeros(480,640, CV_8UC1);
    bitwise_not(fin, cannyInv);
    //    // imshow("cannyInv",cannyInv);//IMSHOW1
    
    vector< vector<Point> > contours;
    findContours(inputIm, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    Mat mask = inputIm;
    // CV_FILLED fills the connected components found
    drawContours(mask, contours, -1, Scalar(255), CV_FILLED);
    //    // imshow("mask", mask);//IMSHOW1
    Mat dist=Mat::zeros(480,640, CV_8UC1);;
    distanceTransform( cannyInv, dist, CV_DIST_L2, 3 );
    
    Mat distNorm;
    
    dist.convertTo(distNorm, CV_8UC1,1,0);
    Mat result= Mat::zeros(480,640, CV_8UC1);
    distNorm.copyTo(result, mask);
    Mat tmp=Mat::zeros(480,640, CV_8UC1);
    Mat fik=Mat::zeros(480,640, CV_8UC3);
    
    //    imwrite(DISTMAP_PATH,result);
    //    imshow("distanceMap", result);waitKey(0);//IMSHOW1
    for(int i = 0; i < result.rows; i++)
    {
        for(int j = 0; j < result.cols; j++)
        {
            if ( result.at< uchar >( i,j ) > 0){
                uchar val = result.at< uchar >( i,j  );
                if(val>maxVal){
                    if(val>0){
                        cv::circle(tmp,cvPoint(j,i),2,255,-1);
                    }
                    maxVal=val;
                    maxX = j;
                    maxY = i;
                }
            }
        }
    }
    cv::circle(result,cvPoint(maxX,maxY),5,255,-1);
    Mat planes[] = {result, result, tmp};
    merge(planes, 3, fik);
    
    //    printf( "x: %i y:%i  val: %d\n",maxX,maxY, maxVal );
    //    imwrite(CENTR_PATH,fik);
    result.release();
    dist.release();
    distNorm.release();
    mask.release();
    cannyInv.release();
    fin.release();
    inputIm.release();
    
    centIndx.first = maxX;
    centIndx.second = maxY;
    return centIndx;
}
/**
 * Geodesic & Optical flow algorithm
 * @param  i1            cv::Mat frame1
 * @param  i2            cv::Mat frame2
 * @param  depthImage    depth image in grayscale
 * @param  option        false no se aplica optical flow
 * @param  voxel_size    size of voxel grid leaf
 * @param  th_moveAround threshold to remove nodes in that radius
 * @param  th_maxflow    threshold of flow vector magnitude
 * @param  saveFlow      save optical flow image to disk or not
 * @param  saveDisk      save resulting image to disk or not
 * @return               Mat image of geoflow with geodesic distanves as px values
 */
Mat geoflow(Mat i1,Mat i2, IplImage* depthImage, bool option,float voxel_size,int th_moveAround,float th_maxflow,bool saveFlow,bool saveDisk,char *glb_filename,kinectData &kd,char *name){
    //--------------------------------------------------------------------------------------------------
    int maxD = -1;
    int minD = MAXIMUM;
    PointXYZRGB centerDM;
    // Build a point cloud (just depth, no color) in which only the user appears  from a pointer to a depth image (unsigned short*). Since
    imageA = i1;
    imageB = i2;
    pairInts centIndx = findCentre(imageB);
    //centIndx.first=273;
    //centIndx.second = 191;
    //    //    imshow("jo",img);//IMSHOW1
    unsigned char* pDepth = (unsigned char*) depthImage->imageData; // pointer first position depth image
    //	unsigned char* pPlayerIndex = (unsigned char*) playerIndex->imageData;	// pointer first position user IDs image
    
    int iter = 0;
    
    //!!!!
    
    float x_rw, y_rw, z_rw; // real world coords
    
    for (int y_p = 0; y_p < yRes; y_p++)
    {
        for (int x_p = 0; x_p < xRes; x_p++)
        {
            iter++;
            int z_p = pDepth[y_p * xRes + x_p]; // projective death. need conversion!
            XnPoint3D pDepthp = xnCreatePoint3D(x_p, y_p, z_p);
            //			projectiveToRealWorldCoordinates(x_p, y_p, z_p, &x_rw, &y_rw, &z_rw);
            XnPoint3D p3d = myConvertProjectiveToRealWorld(pDepthp,kd);
            x_rw = p3d.X;
            y_rw = p3d.Y;
            z_rw = p3d.Z;
            // Fill the user's cloud with user pixels.
            pcl::PointXYZRGB p;
            
            
            p.x = x_rw;
            p.y = y_rw;
            p.z = z_rw;
            
            p.r = z_rw- minD * (255.f/((float)maxD-(float)minD));
            if(p.z > 80 && p.r < 200){
                if(x_p == centIndx.first && y_p == centIndx.second){
                    centerDM = p;
                    
                }
                pCloud->points.push_back(p);
                
            }else{
                PointXYZRGB punt2;
                
                punt2.x = punt2.y = punt2.z = 0;
                punt2.r=punt2.g=punt2.b =punt2.a= 0;
                pCloud->points.push_back(punt2);
            }
        }
    }
    pCloud->height = yRes;
    pCloud->width = xRes;
    pCloud->is_dense = false;
    //    //        visualizePoints(pCloud, "original");      //VIS
    //    printf("processPointsFromFrames::gonna remove high optical flow\n");
    
    
    
    
    if(pCloud->isOrganized()){
        if(option){
            /*-------->*/ removeHighFlow(th_moveAround,th_maxflow,saveFlow);
            //            //                                visualizePoints(pCloud, "unflow");        //VIS
        }
        
        //        printf("processPointsFromFrames::gonna filter with voxel_grid\n");
        /*-------->*/ int sizeVx = down_sample(voxel_size);
        //        //                visualizePoints(pCloudFiltered, "filtred");       //VIS
        
        int croidCntr = vxGrid.getCentroidIndex(centerDM);
        vector < vector< pairInts > > graph = buildGraph(sizeVx);
        //        printf("\n Starting Dijkstra:\n nodes=%d\n", sizeVx);
        int maxGeoVal = dijkstra(croidCntr,sizeVx,graph);
        //        visualizePoints(pCloudFiltered, "dijsktra");        //VIS
        
        MatVec tv = post_process(pCloudFiltered,saveDisk,option,maxGeoVal,vector<pairInts>(),glb_filename, kd,name); //TODO: passing empty vector, correct as in process..
        
        return tv.matr;
    }
    
    Mat m;
    return m;
    //--------------------------------------------------------------------------------------------------
}

Mat imageMD_to_openCVmat(XnUInt16 xRes, XnUInt16 yRes, int which,kinectData &kd){
    kd.g_ImageGenerator.GetMetaData(kd.imageMD);
    const XnUInt8* imageMD_data =kd.imageMD.Data();
    cv::Mat ImgBuf(yRes,xRes,CV_8UC3,(unsigned short*) imageMD_data);
    cv::Mat ImgBuf2;
    
    //    // imshow("kk",ImgBuf2);//IMSHOW1
    
    // IplImage* img = new IplImage(ImgBuf);
    
    cv::cvtColor(ImgBuf,ImgBuf2,CV_RGB2BGR);
    return ImgBuf2;
    
}

Mat getImageNoBackground(kinectData &kd){
    int id_xy;
    
    PointXYZRGB centerDM;
    PointCloud<PointT>::Ptr tmp_cloud (new PointCloud<PointT>);
    for (int y = 0; y < yRes; y++){
        for (int x= 0; x < xRes; x++){
            id_xy = kd.sceneMD(x,y);
            if (  id_xy!=0 ){//depthMD.FrameID()==125 &&
                XnPoint3D pp = xnCreatePoint3D(x, y, kd.depthMD(x,y));
                XnPoint3D prw = myConvertProjectiveToRealWorld(pp,kd);
                PointXYZRGB punt;
                
                punt.x = prw.X;
                punt.y = prw.Y;
                punt.z = prw.Z;
                // punt.r = rgb24Map(x,y).nRed;
                // punt.g = rgb24Map(x,y).nGreen;
                // punt.b = rgb24Map(x,y).nBlue;
                tmp_cloud->points.push_back(punt);
                // imREADY = 1;// to take only one frame
                
            }else{
                //printf("\t\tperdido!\n");
                PointXYZRGB punt2;
                
                punt2.x = punt2.y = punt2.z = 0;
                punt2.r=punt2.g=punt2.b =punt2.a= 0; // # COLOR
                tmp_cloud->points.push_back(punt2);
                
            }//endif
        }//endfor1
        
        
    }//endfor2
    // printf("fin for's\n");
    tmp_cloud->height = yRes;
    tmp_cloud->width = xRes;
    tmp_cloud->is_dense = false;
    
    //    //visualizePoints(tmp_cloud, "tmp_cloud");      //VIS
    
    return saveResult(tmp_cloud,true,kd);
}

PointXYZRGB buildPointCloud(pairInts centrCoords,kinectData &kd){
    //    printf("->buildPointCloud \n");
    int id_xy;
    //    const xn::RGB24Map& rgb24Map = imageMD.RGB24Map();
    PointXYZRGB centerDM; // TODO: check for existance on return
    
    for (int y = 0; y < yRes; y++){
        for (int x= 0; x < xRes; x++){
            id_xy = kd.sceneMD(x,y);
            if (  id_xy!=0 ){//depthMD.FrameID()==125 &&
                XnPoint3D pp = xnCreatePoint3D(x, y, kd.depthMD(x,y));
                XnPoint3D prw = myConvertProjectiveToRealWorld(pp,kd);
                PointXYZRGB punt;
                
                
                punt.x = prw.X;
                punt.y = prw.Y;
                punt.z = prw.Z;
                //                punt.r = 0;
                //                punt.g = 0;
                //                punt.b = 0;
                
                
                if(x == centrCoords.first && y == centrCoords.second){
                    //                    printf("found coordinate coincidence! x:%d,y:%d\n",x,y);
                    centerDM = punt;
                }
                
                pCloud->points.push_back(punt);
                // imREADY = 1;// to take only one frame
                
            }else{
                //printf("\t\tperdido!\n");
                PointXYZRGB punt2;
                
                punt2.x = punt2.y = punt2.z = 0;
                punt2.r=punt2.g=punt2.b =punt2.a= 0; // # COLOR
                pCloud->points.push_back(punt2);
                
            }//endif
        }//endfor1
        
        
    }//endfor2
    // printf("fin for's\n");
    pCloud->height = yRes;
    pCloud->width = xRes;
    pCloud->is_dense = false;
    return centerDM;
}
void depthOpenCVshow(XnUInt16 xRes,XnUInt16 yRes,kinectData &kd){
    //    printf("Getting depthMD\n");
    kd.g_DepthGenerator.GetMetaData(kd.depthMD);
    const XnDepthPixel* depthMD_Data = kd.depthMD.Data();
    //    printf("Done!\n");
    cv::Mat depthMat(480,640,CV_16UC1,(unsigned char*)depthMD_Data);
    cv::Mat depthf(cv::Size(640,480),CV_8UC1);
    depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
    cv::namedWindow("depth",CV_WINDOW_AUTOSIZE);
    cv::imshow("depth",depthf);
    cv::waitKey(0);
}



int processPointsFromFrames(int th_moveAround,float th_maxflow,bool saveFlow,bool saveDisk,int INIFRAME,int FINFRAME,bool b_removeHighFlow,float leafSize, char * glb_filename,kinectData &kd,char *name){
    int num_users = 0;
    frameID = 0;
    PointXYZRGB startPoint;
    vector<float> interMean;
    vector<vector<float> > finalMean;
    bool imAset = false;
    bool imBset = false;
    int maxGeoVal;
    vector<pairInts> oldHands;
    // cojo solo una secuencia hasta FINFRAME
    while ( frameID < FINFRAME && !xnOSWasKeyboardHit())
    {
        frameID = kd.depthMD.FrameID();
        
        // printf("En el bucle\n");
        kd.nRetVal = kd.g_Context.WaitOneUpdateAll(kd.g_DepthGenerator);
        if (kd.nRetVal != XN_STATUS_OK)printf("UpdateData failed: %s\n", xnGetStatusString(kd.nRetVal));
        
        
        // Process the meta data
        kd.g_DepthGenerator.GetMetaData(kd.depthMD);
        kd.g_UserGenerator.GetUserPixels(0, kd.sceneMD);
        kd.g_ImageGenerator.GetMetaData(kd.imageMD);
        num_users = kd.g_UserGenerator.GetNumberOfUsers(); // getNumOfUsers() ??
        //        cout<<"NumUsers:"<<num_users<<endl;
        // cuando detecto primer frame que me interesa de la secuencia
        if(frameID >= INIFRAME){
            cout<<"Frame:"<<frameID<<endl;
            // rgbOpenCVshow(xRes,yRes);
            // depthOpenCVshow(xRes,yRes);
            
            // if (frameID==125 && num_users > 0){
            if ( num_users > 0){
//                printf("FrameID= %d\n", frameID);
                
                if(!imAset){
                    //                    printf("processPointsFromFrames::imagen A asignada\n");
                    imageA = imageMD_to_openCVmat( xRes,yRes, 1,kd);
                    imAset = true;
                    continue;
                }
                if(imAset && !imBset){
                    //                    printf("processPointsFromFrames::imagen B asignada\n");
                    imageB = imageMD_to_openCVmat( xRes,yRes, 2,kd);
                    Mat nob = getImageNoBackground(kd);
                    startPoint = buildPointCloud(findCentreCompl(nob),kd);//geocentr in full cloud
                    //getting starting point coordinates
//                    cout<<"centr = "<<startPoint.x<<endl;
//                    glb_centr = startPoint.x;
                    imBset = true;
                }
                
                
                //                printf("processPointsFromFrames::size pCLoud->points = %ld\n",pCloud->points.size());
                //                                visualizePoints(pCloud,"pCloud");               /***************/ //VIS
                
                if(b_removeHighFlow){
                                         printf("processPointsFromFrames::gonna remove high optical flow\n");
                    /*-------->*/vector<pairInts> pointsToRemove = removeHighFlow(th_moveAround,th_maxflow,saveFlow);
//                                         visualizePoints(pCloud,"removed HighFlow:pCloud");/***************/     //VIS
                }
                //                printf("processPointsFromFrames::gonna filter with voxel_grid\n");
                /*-------->*/ int sizeVx = down_sample(leafSize);
                
                
                int headIndex = vxGrid.getCentroidIndex(startPoint);//geocentr in reduced cloud
                
                if (headIndex < 0) {
                    //                    printf("Cannot found headIndex. Maybe region not exist i.e has been removed.\n moving around...\n");
                }
                while(headIndex < 0){
                    
                    startPoint.y--;
                    headIndex = vxGrid.getCentroidIndex(startPoint);//geocentr in reduced cloud
                }
                
                vector < vector< pairInts > > graph = buildGraph(sizeVx);
                //                printf("\n Starting Dijkstra:\n nodes=%d\n", sizeVx);
                maxGeoVal = dijkstra(headIndex,sizeVx,graph);
                if(maxGeoVal == 0){
                    printf("\nSTOP - something wrong! check maxGeoVal\n");
                    return(frameID);
                }
                
                //                 visualizePoints(pCloudFiltered, "filterWithVoxelG: pCloudFiltered");/***************/       //VIS
                //                printf("processPointsFromFrames::gonna organize voxel_grid\n");
                MatVec tv = post_process(pCloudFiltered,saveDisk,b_removeHighFlow,maxGeoVal,oldHands,glb_filename,kd,name);
                interMean = tv.vec; //TODO if removeHighFlow
                finalMean.push_back(interMean);
                
                oldHands = tv.hands;
                //                printf("oldHands.size= %ld\n",oldHands.size());
                
            }else{printf("\n");}
            
            //            printf("processPointsFromFrames::saving point cloud screenshot..\n");
            
            //                    char filename[512];
            //                    sprintf (filename, "../images/images/frameOrg%d.png", frameID);
            //                    pcl::io::savePNGFile(filename,*pCloud);
            //                    printf("processPointsFromFrames::%s successfully saved\n",filename);
            
            
            
            
            pCloud->clear();
            boost::this_thread::sleep (boost::posix_time::microseconds (600000));
            
        }//endif
        
        //swap images
        imBset=false;
        imageB.copyTo(imageA);
        imageB.release();
        
        
        
    }//endWHILE
    printf("processPointsFromFrames: Exit from while\n");                            //  --->while
    
    
    if(removeHighFlow){
        //        printf("finalMean.size=%ld\n",finalMean.size());
        //looking for longest
        int max=0;
        
        for (int i = 0; i<finalMean.size(); i++) {
            int s = finalMean.at(i).size();
            
            if(s>max){
                max=s;
            }
        }
        //        printf("Longest vector is of %d elements\n",max);
        
        
        //calculating mean for each position
        vector<float> finalVec;
        
        for (int j=0; j<max; j++) {
            float sum = 0.0;
            int cont = 0;
            for (int i = 0; i<finalMean.size(); i++) {
                if(j < finalMean.at(i).size()){
                    sum+=finalMean.at(i).at(j);
                    cont++;
                }
            }
            finalVec.push_back(sum/cont);
        }
        
        ofstream myfile;
        //        myfile.open (FINAL_VEC_PATH);
        
        for (int i = 0; i<finalVec.size(); i++) {
            //            myfile << finalVec.at(i) << endl;
        }
        //        myfile.close();
        //        printf("%ld elements has been written\n",finalVec.size());
        
        
    }
    return 0;
}



int readCAMERA_or_ONI(int opt, char *source, kinectData &kd ){
    // char argv[90] = "/Users/vito/Kinect/oniRecordings/porfinOni.oni";
    
    
    //  [ STAGE 1 ]
    if (opt == 1)
    {
        xn::EnumerationErrors errors;
        kd.nRetVal = kd.g_Context.InitFromXmlFile(SAMPLE_XML_PATH, kd.g_scriptNode, &errors);
        if (kd.nRetVal == XN_STATUS_NO_NODE_PRESENT)
        {
            XnChar strError[1024];
            errors.ToString(strError, 1024);
            printf("%s\n", strError);
            exit (kd.nRetVal);
        }else if (kd.nRetVal != XN_STATUS_OK)      {
            printf("Open failed: %s\n", xnGetStatusString(kd.nRetVal));
            exit (kd.nRetVal);
        }
        printf("Opened from XML\n");
    }   else if(opt == 2)   {
        kd.nRetVal = kd.g_Context.Init();
        // CHECK_RC(nRetVal, "Init");
        kd.nRetVal = kd.g_Context.OpenFileRecording(source, kd.g_Player);
        if (kd.nRetVal != XN_STATUS_OK)
        {
            printf("Can't open recording %s: %s\n", source, xnGetStatusString(kd.nRetVal));
            return 1;
        }else{
            printf("[ok] Recording opened\n");
        }
        
    }
    return 0;
}
int generateDataFromNodes(kinectData &kd){
    //  [ STAGE 4 ]
    
    printf("Starting generating...\n");
    kd.nRetVal = kd.g_Context.StartGeneratingAll();       //Sin esto no hay sceneMD
    CHECK_RC(kd.nRetVal, "StartGenerating");
    
    
    return 0;
}
// --------------------------------------------------------------------------------------------------------------
//                                                  MAIN
// --------------------------------------------------------------------------------------------------------------
int main2(char name[], int ini, int fini)
{
    
    //    if(argc>2){
    //        file1 = argv[1];
    //        file2 = argv[2];
    //        printf("path1: %s, path2: %s\n",file1,file2 );
    //        //    file1 = INPUT_IMGa_PATH;
    //        //    file2 = INPUT_IMGb_PATH;
    //
    //        // char argv[90] = INPUT_ONI_PATH;
    //
    //        findAndPrepareNodesSimple();
    //        Mat i1 = imread(file1,CV_LOAD_IMAGE_GRAYSCALE);
    //        //    imageB  = imread("/Users/vito/Desktop/prision/2.png");
    //        IplImage* depthImage = cvLoadImage(file2,CV_LOAD_IMAGE_GRAYSCALE);//CV_LOAD_IMAGE_GRAYSCALE  -- CV_LOAD_IMAGE_UNCHANGED
    //        Mat i2(depthImage);
    //
    //        //            imshow("i1",i1);        //IMSHOW1
    //        //            imshow("i2",i2); waitKey(0);       //IMSHOW1
    //
    //        geoflow(i1,i2,depthImage,true,3.0,40,4,true,true);
    //        cleanupExit(0);
    //
    //
    //        //1 : camera; 2: oni
    //
    //    }else if(argc == 2){
    //        printf("Gonna read oni data\n");
    //        readCAMERA_or_ONI(2,argv[1]);
    //
    //    }
    //    else{
    //        printf("Gonna read kinect data\n");
    //        readCAMERA_or_ONI(1,NULL);
    //    }
    //    findAndPrepareNodes();
    //    generateDataFromNodes();
    //
    //
    //    //int th_moveAround,int th_maxflow,bool saveFlow,bool saveDisk,int INIFRAME,int FINFRAME
    //    int i = 264;
    //    int j = 1047;
    ////    processPointsFromFrames(3,1,true,true,620,650,true,10.0);//size grid ok 9.0   flow good
    //    while(    i != 1 && i < j){//size grid ok 9.0 no flow
    //        i = processPointsFromFrames(3,1000,true,true,i,j,true,10.0);
    //        i++;
    //    }
    //
    //
    //    cleanupExit(0);
    //
    //
    //    // cout << "Usage: <./main pathImage1 pathImage2>" << endl;
    //
    
    //int th_moveAround,int th_maxflow,bool saveFlow,bool saveDisk,int INIFRAME,int FINFRAME
    int i = ini;
    int j = fini;
    //    processPointsFromFrames(3,1,true,true,620,650,true,10.0);//size grid ok 9.0   flow good
    
    
    vector<pair<int,int> > combinations;
    vector<float> cat1,cat2;
    cat1.push_back(0.01);//%
    cat1.push_back(0.02);
    cat1.push_back(0.03);
    cat1.push_back(0.04);
    cat1.push_back(0.05);
    cat2.push_back(5);//mm
    cat2.push_back(10);
    cat2.push_back(15);
    cat2.push_back(20);
    cat2.push_back(25);
    for( int ix = 3; ix < 5; ix++){//%
     
        for( int id = 0; id < 1; id++){
            i=ini;
            kinectData kd;
            
            char onipath[300];
            sprintf(onipath,"%s%s.oni",DB_PATH,name);
            printf("Gonna read oni data\n %s\n",onipath);
            readCAMERA_or_ONI(2,onipath,kd);
            
            findAndPrepareNodes(kd);
            generateDataFromNodes(kd);
            char glb_filename[500];
            sprintf(glb_filename,"%s%s_difcoord_%.2f_%.2f.txt",DB_PATH,name,cat1.at(ix),cat2.at(id));
            printf("%s\n",glb_filename);
            threshold_mm=cat2.at(id);
            threshold_numpoints=cat1.at(ix);
            while(    i != 1 && i < j){//size grid ok 9.0 no flow
                
                i = processPointsFromFrames(3,1,true,true,i,j,true,10.0,glb_filename,kd,name);
                i++;
            }
        }
    }
//    cleanupExit(0,kd);
    return 0;
}

int main(void){
//    main2("Inta1", 313, 370);
//    main2("Inta4", 709, 1027);
//    main2("bab1", 264, 1047);
//    main2("vit", 651, 900);
    main2("bab2", 334, 721);
//    main2("bengu", 562, 643);
//    main2("ir2", 510, 717);
//    main2("ir3", 867, 1040);
    
    return 0;
}