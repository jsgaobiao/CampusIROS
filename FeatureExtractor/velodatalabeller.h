/** @file
 *  @brief 机器人在线运行框架
 *  @author 鞠孝亮 (JU Xiaoliang)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */
#ifndef VELODATALABELLER_H
#define VELODATALABELLER_H

// LOCAL
#include "grounddetector.h"
#include "lanemarkerdetector.h"

// UTILS
#include <laserscandata.h>
#include <velocalib.h>
#include <logger.h>
#include <navdata.h>
#include <mapdata.h>

// QT
#include <QDebug>

/**
 * @brief 实现Velodyne激光雷达单帧特征提取
 */
class VeloFeatureExtractor {
public:
    VeloFeatureExtractor();
    /**
     * @brief setCalibInfo 设置特征提取所需的Velodyne标定信息
     * @param calibInfo Velodyne标定信息
     */
    void setCalibInfo(VeloCalib &calibInfo);
    /**
     * @brief setRangeMax 设置特征提取对应的激光测距范围
     * @param highRangeMax 高层特征提取最大测距值
     * @param midRangeMax 中层特征提取最大测距值
     * @param gndRangeMax 地面层特征提取测距值在x-y平面的投影最大值
     */
    void setRangeMax(double highRangeMax, double midRangeMax, double gndRangeMax);
    /**
     * @brief setInvalidScope 根据车体大小以及激光雷达与车体的相对位置设置无效激光点范围，均为正数
     * @param front 激光雷达前侧无效范围
     * @param rear 激光雷达后侧无效范围
     * @param left 激光雷达左侧无效范围
     * @param right 激光雷达右侧无效范围
     */
    void setInvalidScope(double front, double rear, double left, double right);
    /**
     * @brief processCurrentFrame 进行当前帧的特征提取，提取结果将按照同样的激光点排列顺序存储在data->label中
     * @param data 激光雷达数据
     */
    void processCurrentFrame(VelodyneRingData &data);
    /**
     * @brief extractScanFeature 提取中、高层或任意非地面指定高度范围的特征
     * @param framedata 激光点云数据
     * @param tmpLMS 特征提取输出结果
     * @param zRange 激光点高度范围,格式：std::pair<double a, double b>,其中a < b，单位：m
     * @param rangMax 最大测距范围，超出该范围的点认为是无效点
     * @param label 对该特征提取结果设定的标签值
     * @param basePlane 是否基于地平面设定zRange，true为基于地平面设置高度，false即根据原始数据z值设定高度
     */
    void extractScanFeature(VelodyneRingData &framedata, LaserScanData &tmpLMS, std::pair<double, double> zRange, double rangMax, Label label, bool basePlane);
    /**
     * @brief extractScanFeature 提取地面特征
     * @param framedata 激光点云数据
     * @param groundFeature 特征提取输出结果
     */
    void extractScanFeature(VelodyneRingData &framedata, LaserScanData &groundFeature);
    /**
     * @brief extractScanFeatureByLabel 从Velodyne特征提取结果中按Label标签值提取LaserScanData
     * @param framedata 激光点云数据
     * @param Feature 特征提取输出结果
     * @param label 标签值
     * @param rangeMax 最大测距范围，超出该范围的点认为是无效点
     */
    void extractScanFeatureByLabel(const VelodyneRingData &framedata, LaserScanData &Feature, Label label, double rangeMax);

    /**
     * @brief pruneInvalidPoint 删除影响匹配的障碍物无效激光点
     * @param data 激光点云数据
     * @param locPos 定位结果预测值
     * @param localizationBelief 最低定位结果匹配置信度，定位结果小于该值时不操作
     * @param mapData 地图数据（Likelihood Map）
     */
    void pruneInvalidPoint(VelodyneRingData &data, NavData locPos, double localizationBelief, const MapData &mapData);
public:
    std::pair<double, double> highFeatureRange;/**< 高层激光特征高度范围*/
    std::pair<double, double> midFeatureRange;/**< 中层激光特征高度范围*/
    LaserScanData highFeature;/**< 高层激光特征*/
    LaserScanData midFeature;/**< 中层激光特征*/
    LaserScanData gndFeature;/**< 地面层激光特征*/
    LaserScanData laneFeature; /**< 车道线激光特征*/

private:
    GroundDetector groundDetector;/**< 地面提取模块类*/
    LaneMarkerDetector laneMarkerDetector;/**< 地面标志检测模块类*/
    //StrangeObjectDetector movObjLabeller;/**< 障碍物剔除模块类*/
    LaserLineSeq smoothSeq;/**< 地面提取得到的平滑曲线段*/
    VeloCalib *calibInfo;/**< 激光雷达标定信息指针*/
    double highRangeMax;/**< 高层激光特征最大测距范围*/
    double midRangeMax;/**< 中层激光特征最大测距范围*/
    double gndRangeMax;/**< 地面层激光特征最大测距范围的x-y平面投影值*/
    double offsetFront, offsetRear, offsetLeft, offsetRight;/**< 激光雷达距离车辆矩形框边界的前、后、左、右距离*/
};

#endif // VELODATALABELLER_H
