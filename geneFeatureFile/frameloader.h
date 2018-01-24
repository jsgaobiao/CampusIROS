/** @file
 *  @brief 离线特征提取
 *  @author 梅继林 (MEI Jilin)
 *  @date 2017.12
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */
#ifndef FRAMELOADER_H
#define FRAMELOADER_H
#include <velodyneringdata.h>
#include <QString>
#include <fstream>

/**
 * @brief The FrameLoader class 点云数据读取接口
 */
class FrameLoader
{
public:
    FrameLoader();
    /**
     * @brief setPath 设置点云数据路径
     * @param path 文件路径
     */
    void setPath(const char *path);

    /**
     * @brief loadOneVeloFrame 获取一帧点云数据
     * @param oneframe 输出 点云数据
     * @return 1-读取成功， 0-读取失败
     */
    bool loadOneVeloFrame(VelodyneRingData &oneframe);

    QString input_xyzi_filename; /**< 输入文件路径>*/
    FILE *logFile;
    int lineNum; /**< 激光雷达线数 该项目设置为64 >*/
    int maxLinePointNum; /**< 每条激光线上点的个数 >*/
};

#endif // FRAMELOADER_H
