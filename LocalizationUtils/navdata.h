/** @file
 *  @brief 机器人在线运行框架.
 *  @author 徐良威 (XU Liangwei)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */

#ifndef NAVDATA_H
#define NAVDATA_H

#include "basedata.h"

/**
 * @brief 位置数据
 *
 */
class NavData : public BaseData {
public:
    NavData();

    double x; /**< x坐标 */
    double y; /**< y坐标 */
    double z; /**< z坐标 */
    double yaw; /**< 以z轴为旋转轴旋转的角度 */
    double pitch; /**< 以y轴为旋转轴旋转的角度 */
    double roll; /**< 以x轴为旋转轴旋转的角度 */
    double confidence; /**< 置信度 */
    double yawRate; /**< yaw角度的变化率 rad/s */
    double velocity; /**< 当前速度 */
    double accuX; /**< x坐标值的精确度 */
    double accuY; /**< y坐标值的精确度 */
    double accuZ; /**< z坐标值的精确度 */
    int status; /**< 状态 */
    std::string scene; /**< 当前所在场景 */
};

#endif // NAVDATA_H
