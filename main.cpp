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

    // Convert to 3-dimention Coordinates
    long long timestamp = lasers[0].time;

    for( const velodyne::Laser& laser : lasers ){

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
        data.points.at(laser.id).push_back(cv::Point3d(x, y, z));
        data.angle.at(laser.id).push_back(laser.azimuth);
        data.distance.at(laser.id).push_back(distance);
        data.intensity.at(laser.id).push_back(laser.intensity);
        data.label.at(laser.id).push_back(Label(Label::Unknown));
        data.enable.at(laser.id).push_back(1);
    }
}

int main( int argc, char* argv[] )
{
    // Open VelodyneVelodyneRingDataCapture that retrieve from PCAP
    const std::string filename = "/media/pku-m/TOSHIBA/Data/20170410_campus/origin/campus2/2017-04-11-09-38-58_Velodyne-HDL-32-Data.pcap";
    const std::string gndfeatfilename = "/media/pku-m/TOSHIBA/Data/20170410_campus/origin/campus2/tst.xy";
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

    std::ofstream gndFeatFile;
    gndFeatFile.open(gndfeatfilename, std::ios_base::binary);

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
        long long lasertime = millsecFromStartOfDay(timestamp);

        timestampfile<<timestamp<<'\t'<<lasertime<<std::endl;

        // Show Ground
        std::vector<cv::Point2d> gndpts;
        for (int i = 0; i < LINE_NUM; i ++)
            for (int j = 0; j < data.points[i].size(); j ++) {
                buffer.push_back( cv::Vec3f( data.points[i][j].x, data.points[i][j].y, data.points[i][j].z ) );
                if (data.label[i][j].is(Label::Ground)) {
                    gndpts.push_back(cv::Point2d(data.points[i][j].x, data.points[i][j].y));
                    bufferColor.push_back(cv::Vec3b(220, 220, 0));
                }
                else {
                    bufferColor.push_back(cv::Vec3b(255, 255, 255));
                }
            }
        //save gnd points
        gndFeatFile.write((char*)&lasertime, sizeof(long long));
        int size = gndpts.size();
        gndFeatFile.write((char*)&size, sizeof(int));
        for (int i=0; i<gndpts.size(); i++)
        {
            gndFeatFile.write((char*)&gndpts[i].x, sizeof(double));
            gndFeatFile.write((char*)&gndpts[i].y, sizeof(double));
        }
        // Create Widget
        cv::Mat cloudMat = cv::Mat( static_cast<int>( buffer.size() ), 1, CV_32FC3, &buffer[0] );
        cv::Mat cloudColorMat = cv::Mat( static_cast<int>( bufferColor.size() ), 1, CV_8UC3, &bufferColor[0] );

        cv::viz::WCloud cloud( cloudMat, cloudColorMat );

        // Show Point Cloud
        viewer.showWidget( "Cloud", cloud );
        viewer.spinOnce();
    }

    gndFeatFile.close();

    // Close All Viewers
    cv::viz::unregisterAllWindows();

    return 0;
}
