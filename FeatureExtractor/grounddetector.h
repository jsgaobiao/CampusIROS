/** @file
 *  @brief 机器人在线运行框架
 *  @author 鞠孝亮 (JU Xiaoliang)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */
#ifndef GROUNDDETECTOR_H
#define GROUNDDETECTOR_H

// UTILS
#include <velodyneringdata.h>
#include <label.h>
#include <mathcalc.hpp>
#include <logger.h>


// LOCAL
#include "laserlineseq.h"
#include "planefitting.h"

/**
 * @brief 实现地面激光点检测
 */
class GroundDetector {
public:
    GroundDetector(int ln = 64);
    /**
     * @brief lineSlideSmooth 对单条激光扫描线做滑动均值滤波(可选)
     * @param linein 输入激光点序列
     * @param lineout 输出激光点序列
     * @param num 滑动窗宽度
     */
    void lineSlideSmooth(std::vector<cv::Point3d>const &linein, std::vector<cv::Point3d> &lineout, int num);
    /**
     * @brief labelSmoothSeqForScans 以滑动窗方差为平滑性度量标准提取激光三维点云中的平滑曲线段集合，输出至类内成员 sq1
     * @param data 激光点云数据
     * @param minpoint 曲线段最小点数
     * @param varwid 计算滑动窗方差的窗口宽度
     */
    void labelSmoothSeqForScans(VelodyneRingData const &data, int minpoint, int varwid);
    /**
     * @brief fitGndPlane 进行地平面参数拟合
     * @param data 激光数据
     */
    void fitGndPlane(const VelodyneRingData &data);
    /**
     * @brief labelGnd 标记地面激光点
     * @param data 激光数据
     */
    void labelGnd(VelodyneRingData &data);
    double gndPlaneParams[4];/**< 地平面参数 */
    LaserLineSeq sqgnd;/**< 地面平滑激光点曲线段 */
    int linenum;/**< 激光线数 */
private:
    double varThres[64];/**< 平滑度阈值，用于筛分平滑曲线段 */
    LaserLineSeq sq1;/**< 平滑激光点曲线段 */
    PlaneFitting planefit;/**< 平面拟合计算对象 */
    double gndPlaneRes;/**< 平面拟合残差 */
};

#endif // GROUNDDETECTOR_H
