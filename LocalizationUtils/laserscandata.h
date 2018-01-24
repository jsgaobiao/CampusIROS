/** @file
 *  @brief 机器人在线运行框架
 *  @author 鞠孝亮 (JU Xiaoliang)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */

#ifndef LASERSCANDATA_H
#define LASERSCANDATA_H

// LOCAL
#include "basedata.h"
#include "label.h"

/**
 * @brief 实现单层或单类激光特征的存储
 */
class LaserScanData : public BaseData {
public:
    LaserScanData();
    int num;/**< 存储激光点数量 */
    int validNum;/**< 存储有效激光点数量 */
    double rangeMax;/**< 存储激光点最大范围 */
    int initialPointNum;/**< 存储激光点来源velodyne数据帧单圈扫描点数 */
    std::vector<double> distances;/**< 存储各个激光点与激光器相对距离 */
    std::vector<std::pair<double, double>> points;/**< 存储激光点在局部坐标系下x-y位置 */
    std::vector<std::pair<int, int>> pointInx;/**< 存储激光点在velodyne原数据帧对应的索引，pair存储内容为线索引与点索引，无效点的索引皆为-1 */
    std::vector<Label>label;/**< 存储激光点标签 */
    /**
     * @brief 清空所有数据
     */
    void clear();
    /**
     * @brief 计算有效激光点数
     */
    void calValidPtNum();
    /**
     * @brief 删除无效激光点
     */
    void eraseInvalidPts();
    /**
     * @brief 实现自采样，采样方式为uniform随机采样
     * @param num 采样数量
     */
    void sampleSelf(int num);
};

#endif // LASERSCANDATA_H
