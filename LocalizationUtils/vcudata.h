/** @file
 *  @brief 机器人在线运行框架.
 *  @author 徐良威 (XU Liangwei)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */

#ifndef VCUDATA_H
#define VCUDATA_H

#include "basedata.h"

/**
 * @brief 从VCU接收到的数据
 *
 */
class VcuData : public BaseData {
public:
    VcuData();
    double speedFL; /**< 左前轮速度 */
    double speedFR; /**< 左后轮速度 */
    double speedRL; /**< 右前轮速度 */
    double speedRR; /**< 右后轮速度 */
    double yawrate; /**< 转向的变化率 rad/s */
    double steerangle; /**< 方向盘转角 */
};

#endif // VCUDATA_H
