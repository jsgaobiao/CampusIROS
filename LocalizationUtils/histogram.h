/** @file
 *  @brief 机器人在线运行框架
 *  @author 鞠孝亮 (JU Xiaoliang)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */

#ifndef HISTOGRAM_H
#define HISTOGRAM_H

//STL
#include <vector>
#include <tuple>
#include <string>
#include <sstream>
/**
 * @brief 实现直方图统计，用于测试定位结果精度
 */
class HistoGram
{
public:
    HistoGram();
    /**
     * @brief 清除所有数据
     */
    void clear();
    /**
     * @brief 向该直方图统计中添加新数据
     * @param value 所添加的新数据，类型为double
     */
    void addValue(double value);
    /**
     * @brief 向该直方图统计中添加新的闭开区间统计范围
     * @param lower 统计范围下界
     * @param upper 统计范围上界
     */
    void addRange(double lower, double upper);
    /**
     * @brief 计算直方图个统计范围频数占总数的百分比
     */
    void calPercentage();
    /**
     * @brief 生成字符串类型统计结果，便于输出
     * @return string 直方图统计详细结果字符串
     */
    std::string geneHistResult();
    long long totalDataNum;/**< 统计总数 */
    double meanValue;/**< 均值 */
    double varValue;/**< 方差 */
private:
    double meanSquareValue;/**< 二阶矩 */
    std::vector<std::tuple<double, double, long long, double>> rangeList;/**< 统计范围下界，统计范围上界，统计频数，统计百分比4元组清单 */
};

#endif // HISTOGRAM_H
