/** @file
 *  @brief 机器人在线运行框架.
 *  @author 徐良威 (XU Liangwei)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */

#ifndef IMAGEDATA_H
#define IMAGEDATA_H

#include "basedata.h"
#include <opencv2/opencv.hpp>

/**
 * @brief 图像数据
 *
 */
class ImageData : public BaseData {
public:
    ImageData();
    ImageData(const ImageData &data);
    cv::Mat image; /**< 用opencv的Mat存储 */
};

#endif // IMAGEDATA_H
