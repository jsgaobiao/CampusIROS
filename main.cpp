#include <TrajCapture.h>
#include "utility.h"

using namespace std;

int main( int argc, char* argv[] )
{
    long long lastTime = 0;

    const std::string filename = "/media/pku-m/TOSHIBA/Data/20170410_campus/origin/campus2/2017-04-11-09-38-58_Velodyne-HDL-32-Data.pcap";
    const std::string trajFileName = "/home/pku-m/SemanticMap/CampusIROS/2m.traj";

    velodyne::HDL32EPcapCapture capture( filename );
    TrajCapture trajCap( trajFileName );

    if( !capture.isOpen() ){
        std::cerr << "Can't open VelodyneCapture." << std::endl;
        return -1;
    }

    VelodyneRingData data(LINE_NUM, MAX_POINTS_PER_LINE);
    GroundDetector gndDetector(32);

    int width = 1200;
    int height = 900;

//    cv::VideoWriter fvVidWriter;
//    fvVidWriter.open("fv.avi", CV_FOURCC('D','I','V','X'), 10, cv::Size(width, height), true);

//    cv::VideoWriter tvVidWriter;
//    tvVidWriter.open("tv.avi", CV_FOURCC('D','I','V','X'), 10, cv::Size(tv_width, tv_height), true);

    while( 1 ){

        // Capture One Rotation Data
        std::vector<velodyne::Laser> lasers;
        capture >> lasers;
        if( lasers.empty() ){
            break;
        }
        // Generate Velodyne Ring data
        generateVelodyneRingData(data, lasers);

        // Label points belongs to the ground
        gndDetector.labelGnd(data);

        // Label Outliers of Velodyne Ring data
//         labelOutliers(data);

        // Convert to 3-dimention Coordinates
        std::vector<cv::Vec3f> buffer;
        std::vector<cv::Vec3b> bufferColor;
        buffer.clear();
        bufferColor.clear();
        long long timestamp = lasers[0].time;
        long long lasertime = millsecFromStartOfDay(timestamp);


        if (lastTime == 0) lastTime = lasertime;
        if (lasertime - lastTime > 200) lasertime = lastTime + 100;

        lastTime = lasertime;

        cv::Mat fv = cv::Mat::zeros(height, width, CV_8UC3);

        for (int i = 0; i < LINE_NUM; i ++)
            for (int j = 0; j < data.points[i].size(); j ++) {
                if (data.label[i][j].is(Label::Ground)) {
                    continue;
                }
                else {
                    frontViewOnMat(fv, data.points[i][j].x, data.points[i][j].y, data.points[i][j].z, 1);
                }
            }


        getTrackOnFrontView(trajCap, lasertime, fv, data);

        cv::imshow("sa", fv);
        char key = cv::waitKey();
        if (key == 'q')
            break;
    }
    return 0;
}
