/** @file
 *  @brief 机器人在线运行框架
 *  @author 鞠孝亮 (JU Xiaoliang)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */
#ifndef PLANEFITTING_H
#define PLANEFITTING_H

// UTILS
#include "velodyneringdata.h"
#include "mathcalc.hpp"
#include "RedSVD.h"

// LOCAL
#include "laserlineseq.h"

//QT
#include <QString>

/**
 * @brief 实现地平面拟合
 */
class PlaneFitting {
public:
    PlaneFitting(int ln = 64) {linenum = ln;}
    /**
     * @brief PlaneFitting
     * @param LaserLineSeq 地面平滑曲线段集合指针
     * @param VelodyneRingData 激光点云帧数据指针
     */
    PlaneFitting(const LaserLineSeq*, const VelodyneRingData*, int ln = 64);
    /**
     * @brief fitPlane 拟合平面
     * @param ransacIter RANSAC循环次数
     * @param toledis 激光点距离平面的距离阈值，小于该值则认为是平面上的点
     * @param reference 粗精度的平面参数数组指针
     */
    void fitPlane(int ransacIter, double toledis, double *reference);
    double optimizedParam[4];/**< 最优参数 */
    bool findFlag;/**< 拟合成功标志 */
    double meanResidual;/**< 拟合残差均值 */
    bool isRANSAC;/**< RANSAC启用标志 */
private:
    /**
     * @brief select3PtFromLineseq 随机选择三个点形成一个平面，用于RANSAC单次抽样
     * @param ptsForFit 候选点集合
     * @param zmaxlimit 选点最大z值限制
     */
    void select3PtFromLineseq(std::vector<cv::Point3d> &ptsForFit, double zmaxlimit);// TODO:x,y,z limit
    const LaserLineSeq* seq;/**< 地面平滑曲线段集合指针 */
    const VelodyneRingData* data;/**< 原始激光点云数据指针 */
    std::vector<LaserCut> limitSeq;/**< 进一步筛选激光线索引得到的地面平滑曲线段集合 */
    int linenum;
};

#endif // PLANEFITTING_H
