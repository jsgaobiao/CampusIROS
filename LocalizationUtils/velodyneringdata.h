/** @file
 *  @brief 机器人在线运行框架.
 *  @author 徐良威 (XU Liangwei) 鞠孝亮（JU Xiaoliang）
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */

#ifndef VELODYNERINGDATA_H
#define VELODYNERINGDATA_H

#include "basedata.h"
#include "label.h"
#include <opencv2/opencv.hpp>

/**
 * @brief velodyne数据
 *
 */
class VelodyneRingData : public BaseData {
public:

    /**
     * @brief 构造一帧激光数据
     *
     * @param lineNum 线数
     * @param maxLinePointNum 每条线最多的点数
     */
    VelodyneRingData(size_t lineNum = 64, int maxLinePointNum = 2200);
    std::vector<std::vector<cv::Point3d>> points;
    std::vector<std::vector<double>> distance;
    std::vector<std::vector<double>> angle; /**< TODO: describe */
    std::vector<std::vector<unsigned char>> intensity;
    std::vector<std::vector<Label>> label;
    std::vector<std::vector<int>> enable;

    /**
     * @brief 清空数据
     *
     */
    void clear();

    /**
     * @brief 计算激光帧中两点的点距
     *
     * @param lineID1 第一个点所在的线的id
     * @param pointIndex1 第一个点的位置
     * @param lineID2 第二个点所在的线的id
     * @param pointIndex2 第二个点的位置
     * @return double 距离
     */
    double calPointsDistance(unsigned int lineID1, unsigned int pointIndex1, unsigned int lineID2, unsigned int pointIndex2);

    /**
     * @brief 获得每条线最多的点数
     *
     * @return int 每条线最多的点数
     */
    int getMaxLinePointNum() const;

    /**
     * @brief 获得线数
     *
     * @return int 线数
     */
    int getLineNum() const;

private:
    int lineNum;
    int maxLinePointNum;
};

#endif // VELODYNERINGDATA_H
