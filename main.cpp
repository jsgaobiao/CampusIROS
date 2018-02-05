#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <time.h>
#include <cstdlib>

#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

#include <VelodynePcapCapture.h>
#include <velodyneringdata.h>
#include <grounddetector.h>
#include <ECSegmentation.h>
#include <TrajCapture.h>

#define LINE_NUM 32
#define MAX_POINTS_PER_LINE 2200
#define sqr(X) ((X)*(X))

using namespace std;

int imgCnt = 0;

double calcDis(cv::Point3d &a, cv::Point3d &b)
{
    return std::sqrt(sqr(a.x - b.x) + sqr(a.y - b.y) + sqr(a.z - b.z));
}

int millsecFromStartOfDay(long long us)
{
    long long ms = us / 1000; //us -> ms
    ms %= 1000;
    time_t tick = us/1e6;
    struct tm *ttime;
    ttime = localtime(&tick);
    ms += ttime->tm_hour*3600000 + ttime->tm_min*60000 + ttime->tm_sec*1000 ;
    return int(ms);
}

void generateVelodyneRingData(VelodyneRingData &data, std::vector<velodyne::Laser> lasers)
{
    data.clear();
    std::vector<double> lut32E = { -30.67, -9.3299999, -29.33, -8.0, -28, -6.6700001, -26.67, -5.3299999, -25.33, -4.0, -24.0, -2.6700001, -22.67, -1.33, -21.33, 0.0, -20.0, 1.33, -18.67, 2.6700001, -17.33, 4.0, -16, 5.3299999, -14.67, 6.6700001, -13.33, 8.0, -12.0, 9.3299999, -10.67, 10.67 };
    std::sort(lut32E.begin(), lut32E.end(), [](const double &x, const double &y){return x < y;});

    // Convert to 3-dimention Coordinates
    long long timestamp = lasers[0].time;

    for( const velodyne::Laser& laser : lasers ){

        int orderID = std::lower_bound(lut32E.begin(), lut32E.end(), laser.vertical) - lut32E.begin();

        const double distance = static_cast<double>( laser.distance ) * 0.002;
        const double azimuth  = laser.azimuth  * CV_PI / 180.0;
        const double vertical = laser.vertical * CV_PI / 180.0;
        float x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
        float y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
        float z = static_cast<float>( ( distance * std::sin( vertical ) ) );

        if( x == 0.0f && y == 0.0f && z == 0.0f ){
            x = std::numeric_limits<float>::quiet_NaN();
            y = std::numeric_limits<float>::quiet_NaN();
            z = std::numeric_limits<float>::quiet_NaN();
        }
        // Velodyne Ring Data
        data.setTimeStamp(TimeStamp(timestamp));
        data.points.at(orderID).push_back(cv::Point3d(x, y, z));
        data.angle.at(orderID).push_back(laser.azimuth);
        data.distance.at(orderID).push_back(distance);
        data.intensity.at(orderID).push_back(laser.intensity);
        data.label.at(orderID).push_back(Label(Label::Unknown));
        data.enable.at(orderID).push_back(1);
    }
}

void generatePCLPointCloudData(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, VelodyneRingData &data)
{
    cloud->clear();

    for (int i = 0; i < LINE_NUM; i ++)
        for (int j = 0; j < data.points[i].size(); j ++)
            if (! data.label[i][j].is(Label::Ground)) {
//                if (! data.label[i][j].is(Label::Inlier)) continue;
                if (data.points[i][j].x != std::numeric_limits<float>::quiet_NaN() &&
                    data.points[i][j].y != std::numeric_limits<float>::quiet_NaN() &&
                    data.points[i][j].z != std::numeric_limits<float>::quiet_NaN())
                {
                    cloud->points.push_back(pcl::PointXYZ(data.points[i][j].x, data.points[i][j].y, data.points[i][j].z));
                }
            }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    printf("Total Points Num: %d\n", cloud->width);
}

void labelOutliers(VelodyneRingData &data)
{
    int slidWindow = 3;
    int slidWindowMax = 30;
    double threshold = 0.1;
    int cnt = 0;

    for (int i = 0; i < LINE_NUM; i ++) {
        cnt = 0;
        for (int j = 1; j < data.points[i].size(); j ++) {
            if (calcDis(data.points[i][j], data.points[i][j-1]) < threshold) {
                cnt ++;
            }
            else {
                if (cnt >= slidWindow && cnt < slidWindowMax) {
                    for (int k = j - 1; k >= j - cnt; k --) {
                        data.label[i][k].set(Label::Inlier);
                    }
                }
                cnt = 0;
            }
        }
        if (cnt >= slidWindow && cnt < slidWindowMax) {
            for (int k = data.points[i].size() - 1; k >= data.points[i].size() - cnt; k --) {
                data.label[i][k].set(Label::Inlier);
            }
        }
    }
}

bool comp(const TrajData a, const TrajData b) { return a.timestamp < b.timestamp; }

bool isInBoundingBox(cv::Point3d pnt, cv::Point3f minBoxPnt, cv::Point3f maxBoxPnt)
{
    if (pnt.x >= minBoxPnt.x && pnt.x <= maxBoxPnt.x)
        if (pnt.y >= minBoxPnt.y && pnt.y <= maxBoxPnt.y)
            if (pnt.z >= minBoxPnt.z && pnt.z <= maxBoxPnt.z) {
                return true;
            }
    return false;
}

void getTrackResult(std::string jpgDir, std::string jpgNegDir, TrajCapture &trajCap, cv::viz::Viz3d &viewer,
                    long long lasertime, std::fstream &tpcOut, VelodyneRingData &data,
                    std::vector<cv::Vec3f> &buffer, std::vector<cv::Vec3b> &bufferColor)
{
    TrajData tmp;
    viewer.removeAllWidgets();
    tmp.timestamp = lasertime;
    int start = std::lower_bound(trajCap.data.begin(), trajCap.data.end(), tmp, comp) - trajCap.data.begin();
    for (int i = start; i < trajCap.data.size(); i ++) {
        // Draw bounding box
        if (trajCap.data[i].timestamp - lasertime > 89) break;
        double _x = - trajCap.data[i].ly;
        double _y = trajCap.data[i].lx;
        if (trajCap.data[i].isPedestrian) {
            cv::viz::WCube newCube(cv::Point3f(_x - 0.6, _y - 1, -2.5), cv::Point3f(_x + 0.9, _y + 0.5, -2.5), true, cv::viz::Color::yellow());
            viewer.showWidget(std::to_string(trajCap.data[i].tno) + "BoundingBox", newCube);
        }

        // Draw tno text
        if (trajCap.data[i].isPedestrian) {
            cv::viz::WText3D tnoText(std::to_string(trajCap.data[i].tno), cv::Point3d(_x, _y, 0), 0.5, true, cv::Scalar(0, 255, 0));
            viewer.showWidget(std::to_string(trajCap.data[i].tno), tnoText);
        }

        // Extract point cloud inside the bounding box
        int cntPnt = 0;
        std::vector<cv::Point3d> pntList;
        std::vector<int> pntIntensityList;
        pntList.clear();
        pntIntensityList.clear();
        for (int j = 0; j < LINE_NUM; j ++) {
            for (int k = 0; k < data.points[j].size(); k ++) {
                if (data.label[j][k].is(Label::Ground)) continue;
                if (isInBoundingBox(data.points[j][k], cv::Point3f(_x - 0.6, _y - 1, -2.5), cv::Point3f(_x + 0.9, _y + 0.5, 0))) {
                    cntPnt ++;
                    pntList.push_back(data.points[j][k]);
                    pntIntensityList.push_back(data.intensity[j][k]);
                    data.label[j][k].set(Label::Tracking);
                }
            }
        }

        // Write point cloud info into tpc file
        if (trajCap.data[i].isPedestrian) {
            tpcOut << trajCap.data[i].timestamp << " " << trajCap.data[i].tno << " " << trajCap.data[i].lx << " " << trajCap.data[i].ly << " ";
            tpcOut << cntPnt << std::endl;
            for (int j = 0; j < cntPnt; j ++) {
                tpcOut << pntList[j].x << " " << pntList[j].y << " " << pntList[j].z << " ";
            }
            tpcOut << std::endl;
        }

        // Write image of point cloud into jpg file
        int imgHeight = 250;
        int imgWidth = 150;
        cv::Mat img(imgHeight, imgWidth, CV_8UC3);
        img = cv::Mat::zeros(imgHeight, imgWidth, CV_8UC3);
        for (int j = 0; j < cntPnt; j ++) {
            int imgX = imgHeight - ((pntList[j].z - (-2.5)) / 2.5 * imgHeight);
            int imgY = (pntList[j].y - (_y - 1)) / 1.5 * imgWidth;
            int imgDepth = (pntList[j].x - (_x - 0.6)) / 1.5 * 255;
            int imgIntensity = pntIntensityList[j];

            buffer.push_back( cv::Vec3f( pntList[j].x, pntList[j].y, pntList[j].z ) );
            bufferColor.push_back(cv::Vec3b(0, 0, 255));

            if (imgX >= 0 && imgX < imgHeight && imgY >=0 && imgY < imgWidth) {
                cv::circle(img, cv::Point(imgY, imgX), 3, cv::Scalar(0, imgDepth, imgIntensity), -1);
            }
        }

        if (trajCap.data[i].isPedestrian) {
            cv::imwrite(jpgDir + std::to_string(imgCnt ++) + "_" + std::to_string(trajCap.data[i].tno) + ".jpg", img);
        }
        else {
            cv::imwrite(jpgNegDir + std::to_string(imgCnt ++) + "_" + std::to_string(trajCap.data[i].tno) + ".jpg", img);
        }
    }
}

int main( int argc, char* argv[] )
{
    double _clusterTolerance;
    int _minClusterSize;
    int _maxClusterSize;
    long long lastTime = 0;

    // Open VelodyneVelodyneRingDataCapture that retrieve from PCAP
    freopen("/home/gaobiao/paraments.xml", "r", stdin);
    scanf("%lf %d %d\n", &_clusterTolerance, &_minClusterSize, &_maxClusterSize);
    const std::string filename = "/home/gaobiao/Data/Campus/2017-04-11-09-38-58_Velodyne-HDL-32-Data.pcap";
    const std::string trajFileName = "/home/gaobiao/Data/Campus/2m.traj";
    const std::string pedestrianFileName = "/home/gaobiao/Data/Campus/2m.tpc";
    const std::string pedestrianWhiteList = "/home/gaobiao/Data/Campus/2m.whitelist";
    const std::string jpgDir = "/home/gaobiao/Data/Campus/2/";
    const std::string jpgNegDir = "/home/gaobiao/Data/Campus/2_n/";
    std::fstream tpcOut(pedestrianFileName, std::ios::out);

    velodyne::HDL32EPcapCapture capture( filename );
    TrajCapture trajCap( trajFileName, pedestrianWhiteList );

    if( !capture.isOpen() ){
        std::cerr << "Can't open VelodyneCapture." << std::endl;
        return -1;
    }

    // Create Viewer
    cv::viz::Viz3d viewer( "Velodyne" );

    // Register Keyboard Callback
    viewer.registerKeyboardCallback(
        []( const cv::viz::KeyboardEvent& event, void* cookie ){
            // Close Viewer
            if( event.code == 'q' && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN ){
                static_cast<cv::viz::Viz3d*>( cookie )->close();
            }
        }
        , &viewer
    );

    std::ofstream timestampfile;
    timestampfile.open("timestamp.txt");

    VelodyneRingData data(LINE_NUM, MAX_POINTS_PER_LINE);
    GroundDetector gndDetector(32);
    ECSegmentation ecs;

    while( !viewer.wasStopped() ){

        // Capture One Rotation Data
        std::vector<velodyne::Laser> lasers;
        capture >> lasers;
        if( lasers.empty() ){
            continue;
        }
        // Generate Velodyne Ring data
        generateVelodyneRingData(data, lasers);

        // Label points belongs to the ground
        gndDetector.labelGnd(data);

        // Label Outliers of Velodyne Ring data
//         labelOutliers(data);

        // Construct ECSegmentation Class
        generatePCLPointCloudData(ecs.cloudPtr, data);
//        ecs.initialize(_clusterTolerance, _minClusterSize, _maxClusterSize);2

        // Convert to 3-dimention Coordinates
        std::vector<cv::Vec3f> buffer;
        std::vector<cv::Vec3b> bufferColor;
        buffer.clear();
        bufferColor.clear();
        long long timestamp = lasers[0].time;
        long long lasertime = millsecFromStartOfDay(timestamp);
        if (lastTime == 0) lastTime = lasertime;
        if (lasertime - lastTime > 200) lasertime = lastTime + 100;

        timestampfile<<timestamp<<'\t'<<lasertime<<std::endl;
        lastTime = lasertime;

        // Show segmentation result2
//        ecs.getSegResult(buffer, bufferColor);

        // Show Tracking result
        getTrackResult(jpgDir, jpgNegDir, trajCap, viewer, lasertime, tpcOut, data, buffer, bufferColor);

        // Show Ground
        for (int i = 0; i < LINE_NUM; i ++)
            for (int j = 0; j < data.points[i].size(); j ++) {
                if (data.label[i][j].is(Label::Ground)) {
//                    buffer.push_back( cv::Vec3f( data.points[i][j].x, data.points[i][j].y, data.points[i][j].z ) );
//                    bufferColor.push_back(cv::Vec3b(220, 220, 0));
                }
                else {
                    if (data.label[i][j].is(Label::Tracking)) continue;
                    int R, G, B;
                    R = 255;
                    G = 255;
                    B = 255;
//                    if (!data.label[i][j].is(Label::Inlier)) {
//                        R = 255; G = 0; B = 0;
//                        continue;
//                    }
                    buffer.push_back( cv::Vec3f( data.points[i][j].x, data.points[i][j].y, data.points[i][j].z ) );
                    bufferColor.push_back(cv::Vec3b(B, G, R));
                }
            }

        buffer.push_back( cv::Vec3f( 0, 0, 0 ) );
        bufferColor.push_back(cv::Vec3b(0, 0, 255));
        // Create Widget
        cv::Mat cloudMat = cv::Mat( static_cast<int>( buffer.size() ), 1, CV_32FC3, &buffer[0] );
        cv::Mat cloudColorMat = cv::Mat( static_cast<int>( bufferColor.size() ), 1, CV_8UC3, &bufferColor[0] );

        cv::viz::WCloud cloud( cloudMat, cloudColorMat );
        cloud.setRenderingProperty(cv::viz::POINT_SIZE, 2.5);

        // Show Point Cloud
        viewer.showWidget( "Cloud", cloud );
//        usleep(100000);
        viewer.spinOnce();
    }

    // Close All Viewers
    cv::viz::unregisterAllWindows();
    tpcOut.close();

    return 0;
}
