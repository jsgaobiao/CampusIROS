#ifndef ECSEGMENTATION_H
#define ECSEGMENTATION_H

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <opencv2/opencv.hpp>
#include <cstdlib>

class ECSegmentation
{
public:
    ECSegmentation();
    void initialize(double _clusterTolerance, int _minClusterSize, int _maxClusterSize);
    void getSegResult(std::vector<cv::Vec3f> &buffer, std::vector<cv::Vec3b> &bufferColor);
    bool isPerson(std::vector<pcl::PointIndices>::const_iterator it);

public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treePtr;
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
};

#endif // ECSEGMENTATION_H
