#ifndef UTILITY_H
#define UTILITY_H

#include <algorithm>
#include <cmath>
#include <fstream>
#include <time.h>
#include <cstdlib>

#include <opencv2/opencv.hpp>

#include <VelodynePcapCapture.h>
#include <velodyneringdata.h>

#include <grounddetector.h>

#include <iostream>
#include <vector>
#include <ECSegmentation.h>
#include "TrajCapture.h"

#define LINE_NUM 32
#define MAX_POINTS_PER_LINE 2200
#define sqr(X) ((X)*(X))


double calcDis(cv::Point3d &a, cv::Point3d &b);

int millsecFromStartOfDay(long long us);

void generateVelodyneRingData(VelodyneRingData &data, std::vector<velodyne::Laser> lasers);

void generatePCLPointCloudData(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, VelodyneRingData &data);

void labelOutliers(VelodyneRingData &data);

void getTrackResult(TrajCapture &trajCap,  long long lasertime, std::fstream &tpcOut, VelodyneRingData &data);

void getTrackOnFrontView(TrajCapture &trajCap,  long long lasertime, cv::Mat& fv, VelodyneRingData &data);

void frontViewProjector(double x, double y, double z, bool offset, int& r, int& c);

void frontViewOnMat(cv::Mat& fv, double x, double y, double z, bool offset);

#endif // UTILITY_H
