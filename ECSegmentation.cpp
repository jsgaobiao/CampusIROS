#include "ECSegmentation.h"
#define INF 21474836

ECSegmentation::ECSegmentation():treePtr(new pcl::search::KdTree<pcl::PointXYZ>),
                                 cloudPtr(new pcl::PointCloud<pcl::PointXYZ>)
{
    // Nothing to do
}

void ECSegmentation::initialize(double _clusterTolerance,
                               int _minClusterSize,
                               int _maxClusterSize)
{
    cluster_indices.clear();
    // Creating the KdTree object for the search method of the extraction
    treePtr->setInputCloud (cloudPtr);
    ec.setClusterTolerance (_clusterTolerance); // 0.02 means 2cm
    ec.setMinClusterSize (_minClusterSize);
    ec.setMaxClusterSize (_maxClusterSize);
    ec.setSearchMethod (treePtr);
    ec.setInputCloud (cloudPtr);
    ec.extract (cluster_indices);
}

bool ECSegmentation::isPerson(std::vector<pcl::PointIndices>::const_iterator it)
{
    double minX = INF;
    double minY = INF;
    double minZ = INF;
    double maxX = -INF;
    double maxY = -INF;
    double maxZ = -INF;

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
        double x = cloudPtr->points[*pit].x;
        double y = cloudPtr->points[*pit].y;
        double z = cloudPtr->points[*pit].z;
        minX = x < minX ? x : minX;
        minY = y < minY ? y : minY;
        minZ = z < minZ ? z : minZ;
        maxX = x > maxX ? x : maxX;
        maxY = y > maxY ? y : maxY;
        maxZ = z > maxZ ? z : maxZ;
    }
    if (maxZ - minZ < 0.6 || maxZ - minZ > 2.0) return false;
    if (maxX - minX > 1.8 || maxY - minY > 1.8) return false;
    if (maxZ > -0.1 || maxZ < -1.0) return false;
    return true;
}

void ECSegmentation::getSegResult(std::vector<cv::Vec3f> &buffer, std::vector<cv::Vec3b> &bufferColor)
{
    int objCnt = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        int B = std::rand() % 250;
        int G = std::rand() % 250;
        int R = std::rand() % 250;
        if (! isPerson(it)) {
            continue;
            B = 255;
            G = 0;
            R = 0;
        }
        else {
            B = 0;
            G = 0;
            R = 255;
        }
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
            buffer.push_back(cv::Vec3f(cloudPtr->points[*pit].x, cloudPtr->points[*pit].y, cloudPtr->points[*pit].z));    
            bufferColor.push_back(cv::Vec3b(B, G, R));
        }
        objCnt ++;
    }
    printf("Obj Num: %d\n", objCnt);
}
