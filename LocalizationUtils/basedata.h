/** @file
 *  @brief 机器人在线运行框架.
 *  @author 徐良威 (XU Liangwei)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */

#ifndef BASEDATA_H
#define BASEDATA_H

// LOCAL
#include "timestamp.h"
#include "typeconverter.h"

// STL
#include <iostream>
#include <string>
#include <typeinfo>
#include <cmath>

using std::string;

/**
 * @brief 最基本的数据类，包含事件时间戳
 *
 * 是所有可在文件流中传输的数据的基类
 *
 */
class BaseData {
public:
    BaseData();
    virtual ~BaseData();
    /**
     * @brief 获得时间戳
     *
     * @return TimeStamp 数据中存储的时间戳
     */
    TimeStamp getTimeStamp() const;

    /**
     * @brief 数据的用户可读模式，可自定义
     *
     * @return string
     */
    virtual string toString();

    /**
     * @brief 给数据设置当前时间戳
     *
     * @return TimeStamp 当前时间戳
     */
    TimeStamp setTimeStamp();

    /**
     * @brief 给数据设置时间戳
     *
     * @param value 给定的时间戳
     */
    void setTimeStamp(const TimeStamp &value);
private:
    TimeStamp timeStamp;
};

#endif // BASEDATA_H
