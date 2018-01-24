/** @file
 *  @brief 机器人在线运行框架.
 *  @author 徐良威 (XU Liangwei)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */

#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <string>
#include <iostream>
#include <chrono>
using std::string;

#include "typeconverter.h"

/**
 * @brief 时间戳
 *
 * 使用std::chrone生成时间戳，跨平台可用
 *
 */
class TimeStamp {
public:

    /**
     * @brief 默认构造函数，生成当前时间
     *
     */
    TimeStamp();

    /**
     * @brief 以数字初始化时间戳
     *
     * @param llTimeStamp 时间戳，以微秒为单位
     */
    TimeStamp(long long llTimeStamp);

    /**
     * @brief 以数字初始化时间戳
     *
     * @param nSec 时间戳的秒部分
     * @param nUsec 时间戳的微秒部分
     */
    TimeStamp(int nSec, int nUsec);

    /**
     * @brief 生成当前时间
     *
     * @return TimeStamp 当前时间
     */
    static TimeStamp getCurrentTime();

    /**
     * @brief 生成可阅读的格式
     *
     * @return string 可阅读的字符串
     */
    string toString();

    /**
     * @brief 小于号
     *
     */
    bool operator <(const TimeStamp& other) const;

    /**
     * @brief 大于号
     *
     */
    bool operator >(const TimeStamp& other) const;

    /**
     * @brief 等于号
     *
     */
    bool operator ==(const TimeStamp& other) const;

    /**
     * @brief 不等于号
     *
     */
    bool operator !=(const TimeStamp& other) const;

    /**
     * @brief 减法
     *
     * @return long long 以微妙为单位的差
     */
    long long operator -(const TimeStamp &other);

    /**
     * @brief 获得数字形式的时间戳
     *
     * @return long long 微秒为单位的数字时间戳
     */
    long long getTickCount() const;

    /**
     * @brief 设置时间戳
     *
     * @param value 微秒为单位的数字
     */
    void setTimeStamp(long long value);

    /**
     * @brief 设置时间戳
     *
     * @param sec 时间戳的秒部分
     * @param usec 时间戳的微秒部分
     */
    void setTimeStamp(int sec, int usec);

    /**
     * @brief 获得时间戳的微秒部分
     *
     * @return int 时间戳的微秒部分
     */
    int getNUsec() const;

    /**
     * @brief 获得时间戳的秒部分
     *
     * @return int 时间戳的秒部分
     */
    int getNSec() const;

private:
    int nSec, nUsec;
    long long llTimeStamp;
};

#endif // TIMESTAMP_H
