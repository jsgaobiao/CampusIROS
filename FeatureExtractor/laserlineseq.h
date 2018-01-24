/** @file
 *  @brief 机器人在线运行框架
 *  @author 鞠孝亮 (JU Xiaoliang)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */
#ifndef LASERLINESEQ_H
#define LASERLINESEQ_H

// UTILS
#include <velodyneringdata.h>

// STL
#include <vector>

/**
 * @brief 用于描述一段连续激光扫描点
 *
 */
class LaserCut {
public:
    LaserCut() {
        label = 0;
        averH = 0;
        disvar = -1;
    }
    unsigned int start;/**< 起始点索引 */
    unsigned int end;/**< 终止点索引 */
    int lineID;/**< 所在激光线索引 */
    bool valid;/**< 有效性标志 */
    double averH;/**< 平均点高度，一般为距地面高度 */
    double disvar;/**< 点测距值方差 */
    int label;/**< 标签值 */
};

/**
 * @brief 用于描述多段连续激光点
 *
 */
class LaserLineSeq : public std::vector<LaserCut> {
public:
    LaserLineSeq(int ln = 64);
    /**
     * @brief convert2DataSeq 提供输出单类数据的函数接口，如可仅抽出激光点强度值
     * @param data 激光数据
     * @param xyzitype 所要抽出的数据类型，可为'x','y','z','i',分别表示局部坐标系下的x，y，z值与反射强度值
     */
    void convert2DataSeq(VelodyneRingData const &data, std::vector<std::vector<float>> &, char xyzitype);
    /**
     * @brief sort 按所在激光线索引对本线段序列进行从小到大排序
     */
    void sort();
    int lineNum;/**< 激光雷达线数 */
    int linePtsNum;/**< 激光线点数 */
};

#endif // LASERLINESEQ_H
