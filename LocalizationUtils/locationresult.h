/** @file
 *  @brief 机器人在线运行框架
 *  @author 鞠孝亮 (JU Xiaoliang)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */

#ifndef LOCATIONRESULT_H
#define LOCATIONRESULT_H

// LOCAL
#include "basedata.h"
#include "navdata.h"

/**
 * @brief 存储定位结果及其当前帧必要信息，并可进行实时收发
 */
class LocationResult : public NavData {
public:
    LocationResult();
    /*
     * 主要定位结果相关信息
     */
    double resultEvaluation[3];/**< 定位结果对三层地图对数匹配概率 */
    double slamGTEvaluation[3];/**< SLAM真值结果对三层地图对数匹配概率 */
    double slamResult[3];/**< SLAM真值结果 */
    NavData gps;/**< GPS数据 */
    double bestParticleBelief[3];/**< 三层地图对应的最佳匹配粒子对数匹配概率 */
    int featurePtsNum[3];/**< 当前帧使用的各层特征点数量 */
    double mapWeight[3];/**< 当前帧三层地图权重 */
    int finalParticleNum;/**< 当前帧使用粒子数 */

    /*
     * 与当前定位帧对应的各组数据时间戳
     */
    TimeStamp vcuTime;/**< VCU数据(经过定位结果重置)时间戳 */
    TimeStamp vcuRawTime;/**< VCU数据时间戳 */
    TimeStamp slamPosTime;/**< SLAM数据时间戳 */
    TimeStamp gpsPosTime;/**< GPS数据时间戳 */
    TimeStamp hLaserTime;/**< 激光数据时间戳 */
    double resultClusterCov[3][3];/**< 定位结果所在聚类的协方差 */
    /*
     * 标志位
     */
    bool resampled;/**< 当前帧是否重采样 true为重采样 */
    bool pfConverged;/**< 当前帧粒子滤波是否收敛 true为收敛 */
    bool featureValid;/**< 当前帧是否存在有效特征 true为存在 */
    bool pfRetrieve;/**< 当前帧定位是否恢复正常 true为定位正常 */
    /*
     * 与GPS定位相对误差
     */
    double error2GPS;/**< 到GPS定位点距离 单位:m */
    double error2GPS_lateral;/**< 到GPS定位点横向距离 单位:m */
    double error2GPS_longti;/**< 到GPS定位点纵向距离 单位:m */
    double error2GPS_yaw;/**< 到GPS定位点航向角误差 单位:rad */

    /*
     * 与GPS定位相对误差的统计值
     */
    double meanError2GPS;/**< 截止到当前帧与GPS定位点距离误差均值 单位:m */
    double meanError2GPS_lateral;/**< 截止到当前帧与GPS定位点距离横向误差均值 单位:m */
    double meanError2GPS_longtitude;/**< 截止到当前帧与GPS定位点距离纵向误差均值 单位:m */
    double meanAbsError2GPS_lateral;/**< 截止到当前帧与GPS定位点距离横向误差绝对值均值 单位:m */
    double meanAbsError2GPS_longtitude;/**< 截止到当前帧与GPS定位点距离纵向误差绝对值均值 单位:m */
    double varError2GPS;/**< 截止到当前帧与GPS定位误差方差 单位:m^2 */
    double varError2GPS_lateral;/**< 截止到当前帧与GPS定位横向误差方差 单位:m^2 */
    double varError2GPS_longtitude;/**< 截止到当前帧与GPS定位纵向误差方差 单位:m^2 */
    double varAbsError2GPS_lateral;/**< 截止到当前帧与GPS定位横向误差绝对值方差 单位:m^2 */
    double varAbsError2GPS_longtitude;/**< 截止到当前帧与GPS定位纵向误差绝对值方差 单位:m^2 */
    double velocity;/**< 当前帧对应车速 单位:m/s */
    long long totalFrameNum;/**< 当前总帧数 */

};

#endif // LOCATIONRESULT_H
