#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

#include <VelodynePcapCapture.h>
#include <velodyneringdata.h>
#include <grounddetector.h>

#define LINE_NUM 32
#define MAX_POINTS_PER_LINE 2200

using namespace std;

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

        data.setTimeStamp(TimeStamp(timestamp));
        data.points.at(orderID).push_back(cv::Point3d(x, y, z));
        data.angle.at(orderID).push_back(laser.azimuth);
        data.distance.at(orderID).push_back(distance);
        data.intensity.at(orderID).push_back(laser.intensity);
        data.label.at(orderID).push_back(Label(Label::Unknown));
        data.enable.at(orderID).push_back(1);
    }

}

int main( int argc, char* argv[] )
{
    // Open VelodyneVelodyneRingDataCapture that retrieve from PCAP
    const std::string filename = "/media/gaobiao/SeagateBackupPlusDrive/Campus/origin/campus2/2017-04-11-09-38-58_Velodyne-HDL-32-Data.pcap";
    velodyne::HDL32EPcapCapture capture( filename );

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


    while( !viewer.wasStopped() ){
        // Capture One Rotation Data
        std::vector<velodyne::Laser> lasers;
        capture >> lasers;
        if( lasers.empty() ){
            continue;
        }
        data.clear();
        generateVelodyneRingData(data, lasers);
        gndDetector.labelGnd(data);

        // Convert to 3-dimention Coordinates
        std::vector<cv::Vec3f> buffer( lasers.size() );
        std::vector<cv::Vec3b> bufferColor( lasers.size() );
        long long timestamp = lasers[0].time;
        timestampfile<<timestamp<<'\t'<<millsecFromStartOfDay(timestamp)<<std::endl;

        // Show Ground
        for (int i = 0; i < LINE_NUM; i ++)
            for (int j = 0; j < data.points[i].size(); j ++) {
                if (data.label[i][j].is(Label::Ground)) {
//                    buffer.push_back( cv::Vec3f( data.points[i][j].x, data.points[i][j].y, data.points[i][j].z ) );
//                    bufferColor.push_back(cv::Vec3b(220, 220, 0));
                }
                else {
                    buffer.push_back( cv::Vec3f( data.points[i][j].x, data.points[i][j].y, data.points[i][j].z ) );
                    bufferColor.push_back(cv::Vec3b(255, 255, 255));
                }
            }

        // Create Widget
        cv::Mat cloudMat = cv::Mat( static_cast<int>( buffer.size() ), 1, CV_32FC3, &buffer[0] );
        cv::Mat cloudColorMat = cv::Mat( static_cast<int>( bufferColor.size() ), 1, CV_8UC3, &bufferColor[0] );

        cv::viz::WCloud cloud( cloudMat, cloudColorMat );

        // Show Point Cloud
        viewer.showWidget( "Cloud", cloud );
        viewer.spinOnce();
    }

    // Close All Viewers
    cv::viz::unregisterAllWindows();

    return 0;
}
