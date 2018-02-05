#include "utility.h"


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

bool isInBoundingBox(cv::Point3d pnt, cv::Point3f minBoxPnt, cv::Point3f maxBoxPnt)
{
    if (pnt.x >= minBoxPnt.x && pnt.x <= maxBoxPnt.x)
        if (pnt.y >= minBoxPnt.y && pnt.y <= maxBoxPnt.y)
            if (pnt.z >= minBoxPnt.z && pnt.z <= maxBoxPnt.z) {
                return true;
            }
    return false;
}
bool comp(const TrajData a, const TrajData b) { return a.timestamp < b.timestamp; }
void getTrackResult(TrajCapture &trajCap,  long long lasertime, std::fstream &tpcOut, VelodyneRingData &data)
{
    TrajData tmp;

    tmp.timestamp = lasertime;
    int start = std::lower_bound(trajCap.data.begin(), trajCap.data.end(), tmp, comp) - trajCap.data.begin();


    int width = 1200;
    int height = 900;

    for (int i = start; i < trajCap.data.size(); i ++) {
        // Draw bounding box
        if (trajCap.data[i].timestamp - lasertime > 89) break;
        double _x = - trajCap.data[i].ly;
        double _y = trajCap.data[i].lx;

        // Extract point cloud inside the bounding box
        int cntPnt = 0;
        std::vector<cv::Point3d> pntList;
        pntList.clear();
        for (int j = 0; j < LINE_NUM; j ++) {
            for (int k = 0; k < data.points[j].size(); k ++) {
                if (data.label[j][k].is(Label::Ground)) continue;
                if (isInBoundingBox(data.points[j][k], cv::Point3f(_x - 0.6, _y - 1, -2), cv::Point3f(_x + 0.9, _y + 0.5, 0))) {
                    cntPnt ++;
                    pntList.push_back(data.points[j][k]);
                }
            }
        }
        if (cntPnt < 40) continue;
        tpcOut << trajCap.data[i].timestamp << " " << trajCap.data[i].tno << " " << trajCap.data[i].lx << " " << trajCap.data[i].ly << " ";
        tpcOut << cntPnt << std::endl;
        for (int j = 0; j < cntPnt; j ++) {
            tpcOut << pntList[j].x << " " << pntList[j].y << " " << pntList[j].z << " ";
        }
        tpcOut << std::endl;
    }
}


void getTrackOnFrontView(TrajCapture &trajCap,  long long lasertime, cv::Mat& fv, VelodyneRingData &data)
{
    TrajData tmp;

    tmp.timestamp = lasertime;
    int start = std::lower_bound(trajCap.data.begin(), trajCap.data.end(), tmp, comp) - trajCap.data.begin();

    std::cout<<"laser time: "<<lasertime<<" Traj time: "<<trajCap.data[start].timestamp<<std::endl;
    for (int i = start; i < trajCap.data.size(); i ++) {
        // Draw bounding box
        if (trajCap.data[i].timestamp - lasertime > 89) break;
        double _x = - trajCap.data[i].ly;
        double _y = trajCap.data[i].lx;
        // Extract point cloud inside the bounding box
        int cntPnt = 0;
        std::vector<cv::Point3d> pntList;
        pntList.clear();
        for (int j = 0; j < LINE_NUM; j ++) {
            for (int k = 0; k < data.points[j].size(); k ++) {
                if (data.label[j][k].is(Label::Ground)) continue;
                if (isInBoundingBox(data.points[j][k], cv::Point3f(_x - 0.6, _y - 1, -2), cv::Point3f(_x + 0.9, _y + 0.5, 0))) {
                    cntPnt ++;
                }
            }
        }
        if (cntPnt < 40) continue;

        double x = _x;
        double y = _y-0.5;
        double z = -1.5;

        int r, c;
        frontViewProjector(x,y,z,1, r, c);

        int col = c  + fv.cols/2;
        int row = r + fv.rows/2;

        row = fv.rows - row;

        if (row >=fv.rows || row<0 || col<0 || col>=fv.cols)
            continue;

        cv::circle(fv, cv::Point(col,row), 3, cv::Scalar(255,0,255), 2);
        cv::putText(fv, std::to_string(trajCap.data[i].tno), cv::Point(col,row), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255,255,255), 2);
    }
}

void frontViewProjector(double x, double y, double z, bool offset, int& r, int& c)
{
    double d_theta = CV_PI/400.0;
    double d_phi = CV_PI/800.0;

    double new_z = z;
    if (offset)
    {
        new_z -= 1.5;
    }
    //旋转90度
    c = (atan2(x,-y))/d_theta;
    r = atan2(new_z, std::sqrt(x*x+y*y))/d_phi;
}

void frontViewOnMat(cv::Mat& fv, double x, double y, double z, bool offset)
{
    int r, c;
    frontViewProjector(x,y,z, offset, r, c);

    int B, G, R;
    if (offset)
        R = std::min<int>(255, std::max<int>(0, (z + 3.5)*60));
    else
        R = std::min<int>(255, std::max<int>(0, (z+2.5)*90));

    B = 255 - R;
    if (R>=128)
        G = B*2;
    else
        G = R*2;

    int col = c  + fv.cols/2;
    int row = r + fv.rows/2;

    row = fv.rows - row;

    if (row >=fv.rows || row<0 || col<0 || col>=fv.cols)
        return ;

//    fv.at<cv::Vec3b>(row, col) = cv::Vec3b(B,G,R);
    cv::circle(fv, cv::Point(col, row), 1, cv::Scalar(B,G,R), 1);
}

