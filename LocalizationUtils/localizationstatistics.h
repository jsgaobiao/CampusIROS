/** @file
 *  @brief 机器人在线运行框架
 *  @author 鞠孝亮 (JU Xiaoliang)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */

#ifndef LOCALIZATIONSTATISTICS_H
#define LOCALIZATIONSTATISTICS_H

// LOCAL
#include "histogram.h"
#include "locationresult.h"

// STL
#include <cmath>

/**
 * @brief 实现定位结果精度的直方图统计
 */
class LocalizationStatistics{
public:
    LocalizationStatistics();
    HistoGram error2GPSHist;/**< 到GPS定位点距离直方图 */
    HistoGram error2GPS_lateralHist;/**< 到GPS定位点横向距离直方图 */
    HistoGram error2GPS_longtitudeHist;/**< 到GPS定位点纵向距离直方图 */
    HistoGram error2GPS_yawHist;/**< 到GPS定位点航向角偏差直方图 */
    HistoGram absError2GPS_yawHist;/**< 到GPS定位点航向角偏差绝对值直方图 */
    HistoGram absError2GPS_lateralHist;/**< 到GPS定位点横向距离绝对值直方图 */
    HistoGram absError2GPS_longtitudeHist;/**< 到GPS定位点纵向距离绝对值直方图 */
    /**
     * @brief 更新各个直方图统计
     * @param locationResult 需要添加至统计中去的定位结果
     */
    void update(LocationResult const& locationResult);
    /**
     * @brief 设置默认的各个直方图统计范围
     */
    void setDefaultRangeLists();
};

#endif // LOCALIZATIONSTATISTICS_H
