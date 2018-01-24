/** @file
 *  @brief 机器人在线运行框架
 *  @author 鞠孝亮 (JU Xiaoliang)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */
#ifndef LANEMARKERDETECTOR_H
#define LANEMARKERDETECTOR_H

// UTILS
#include <velodyneringdata.h>
#include <mathcalc.hpp>

// STL
#include <algorithm>

// LOCAL
#include "laserlineseq.h"

/**
 * @brief 实现基于反射强度的车道线激光点检测，检测思想为包络峰值检测
 */
class LaneMarkerDetector {
public:
    LaneMarkerDetector();
    /**
     * @brief labelLaneMarker 标记地面激光点中的高亮度车道线点
     * @param data 激光数据
     * @param sq1 地面激光点曲线段序列
     */
    void labelLaneMarker(VelodyneRingData &data, const LaserLineSeq &sq1);
    /**
     * @brief detectLaneMarker 检测地面激光点中的高亮度车道线点
     * @param data 激光数据
     * @param sq1 地面激光点曲线段序列
     */
    void detectLaneMarker(VelodyneRingData const &data, LaserLineSeq const &sq1);
private:
    LaserLineSeq sqLaneMarker;/**< 以曲线段形式存储检测到的车道线激光点 */
    /**
     * @brief detectSignalPeak 检测连续信号峰
     * @param insignal 输入信号
     * @param pointIDlist 输入信号对应的点索引
     * @param thresFactor 筛选阈值倍数
     * @param influence 惯性滤波因子
     * @param averwid 滑动窗宽度
     * @param outsignal 输出信号
     */
    void detectSignalPeak(std::vector<float> &insignal, std::vector<int> &pointIDlist, float thresFactor, float influence, int averwid, std::vector<int> &outsignal);
};

#endif // LANEMARKERDETECTOR_H
