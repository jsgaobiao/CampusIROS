#include "locallaserstatsmap.h"

LocalLaserStatsMap::LocalLaserStatsMap() {
    pixelSize = 0.5;
    maxDis = 30;
    mapHeight = maxDis / pixelSize * 2;
    mapWidth = maxDis / pixelSize * 2;
    Fmap.create(mapHeight, mapWidth, CV_32FC4);
    Fmap.setTo(0);
    centerX = 0;
    centerY = 0;
    zeroX = mapWidth / 2;
    zeroY = mapHeight / 2;
}

void LocalLaserStatsMap::updateMap(const VelodyneRingData &data, bool usingPlane) {
    Fmap = cv::Scalar::all(0);
    for (size_t i = 0; i != data.points.size(); i++) {
        for (size_t j = 0; j != data.points[i].size(); j++) {
            double x = data.points[i][j].x;
            double y = data.points[i][j].y;
            double z = data.points[i][j].z;
            if (x == 0 || y == 0 || z == 0) {
                continue;
            }
            int ix = zeroX + (x / pixelSize - centerX);
            int iy = zeroY + (y / pixelSize - centerY);
            if (ix > mapWidth - 1 || ix < 0 || iy < 0 || iy > mapHeight - 1) {
                continue;
            }
            if (Fmap.at<cv::Vec4f>(iy, ix)[0] == 0) {
                Fmap.at<cv::Vec4f>(iy, ix)[1] = 100;// 存最小值
                Fmap.at<cv::Vec4f>(iy, ix)[2] = -100;// 存最大值
                Fmap.at<cv::Vec4f>(iy, ix)[3] = 0.0;
            }
            Fmap.at<cv::Vec4f>(iy, ix)[0] += 1;// 存击中点数

            double dis;
            if (usingPlane) {
                dis = gndpara[0] * x + gndpara[1] * y + gndpara[2] * z + gndpara[3];
                if (gndpara[2] < 0) {
                    dis = -dis;
                }
            } else {
                dis = z;
            }
            Fmap.at<cv::Vec4f>(iy, ix)[3] = Fmap.at<cv::Vec4f>(iy, ix)[3] * Fmap.at<cv::Vec4f>(iy, ix)[0] / (Fmap.at<cv::Vec4f>(iy, ix)[0] + 1) + dis / (Fmap.at<cv::Vec4f>(iy, ix)[0] + 1);
            if (dis > Fmap.at<cv::Vec4f>(iy, ix)[2]) {
                Fmap.at<cv::Vec4f>(iy, ix)[2] = dis;
            }
            if (dis < Fmap.at<cv::Vec4f>(iy, ix)[1]) {
                Fmap.at<cv::Vec4f>(iy, ix)[1] = dis;
            }
        }
    }
}
