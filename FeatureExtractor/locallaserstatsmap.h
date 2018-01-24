/** @file
 *  @brief 机器人在线运行框架
 *  @author 鞠孝亮 (JU Xiaoliang)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */
#ifndef LOCALLASERSTATSMAP_H
#define LOCALLASERSTATSMAP_H

// UTILS
#include <velodyneringdata.h>

// OPENCV
#include <opencv2/opencv.hpp>

// STL
#include <vector>

/**
 * @brief 实现局部激光雷达坐标系栅格地图内激光点的统计，如统计某一栅格内的最大激光点高度（暂未使用）
 */
class LocalLaserStatsMap {
public:
    LocalLaserStatsMap();
    cv::Mat Fmap;
    int centerX; /**< 车体所在像素位置 */
    int centerY;
    int zeroX; /**< 零点坐标对应的像素位置 */
    int zeroY;
    double pixelSize; /**< 像素尺寸 单位：m */
    int maxDis; /**< 最大测距值 */
    int mapHeight; /**< 地图尺寸 */
    int mapWidth;
    double *gndpara; /**< 地平面参数 */
    /**
     * @brief updateMap 更新地图
     * @param data 激光数据
     * @param usingplane 是否使用地平面参数信息
     */
    void updateMap(VelodyneRingData const &data, bool usingplane);
};

#endif // LOCALLASERSTATSMAP_H
