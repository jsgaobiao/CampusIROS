/** @file
 *  @brief 离线特征提取
 *  @author 梅继林 (MEI Jilin)
 *  @date 2017.12
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */

#ifndef SAVEFEATURE_H
#define SAVEFEATURE_H

#include "velodatalabeller.h"
#include "velocalib.h"
#include <string>

/**
 * @brief The saveFeature class 特征提取接口类
 */
class saveFeature
{
public:
    saveFeature();
    ~saveFeature();
    /**
     * @brief geneFeatureFiles 从点云数据提取特征
     * @param velodata 激光点云数据
     */
    void geneFeatureFiles(VelodyneRingData &velodata);

    /**
     * @brief setCalibPath 设置Velodyne标定文件路径
     * @param file 文件名
     */
    void setCalibPath(const char *file);

    void setPath(const char *file);

    VeloFeatureExtractor labeller; /**< 特征提取 >*/
    VeloCalib calibParams; /**< 标定文件 >*/
    std::string filePath;

    double getHighRangeMax() const;
    void setHighRangeMax(double value);

    double getMidRangeMax() const;
    void setMidRangeMax(double value);

    double getGndRangeMax() const;
    void setGndRangeMax(double value);

private:
    double highRangeMax; /**< 高层激光数据的最大范围 >*/
    double midRangeMax; /**< 中层激光数据的最大范围 >*/
    double gndRangeMax; /**< 地面激光数据的最大范围 >*/
};

#endif // SAVEFEATURE_H
