/** @file
 *  @brief 机器人在线运行框架.
 *  @author 徐良威 (XU Liangwei)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */

#ifndef PARTICLESETDATA_H
#define PARTICLESETDATA_H

#include "basedata.h"
#include "navdata.h"

/**
 * @brief 存储生成的粒子集合，为位置数据的列表
 *
 */
class ParticleSetData : public BaseData, public std::vector<NavData> {
public:
    ParticleSetData();
};

#endif // PARTICLESETDATA_H
