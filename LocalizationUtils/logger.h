/** @file
 *  @brief 机器人在线运行框架.
 *  @author 徐良威 (XU Liangwei)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */

#ifndef LOGGER_H
#define LOGGER_H

// STL
#include <chrono>
#include <string>
#include <map>
#include <memory>
#include <fstream>

// QT
#include <QString>
#include <QDebug>

// UTILS
#include <timestamp.h>

/**
 * @brief 用于日志输出的各种函数
 *
 */
namespace Logger {
static bool enableTimePrint = false; /**< 是否允许把时间输出到命令行 */
static std::map<std::string, TimeStamp> timeLogStarttime; /**< 存储开始时间 */

static bool enableLogPrint = false; /**< 是否允许把日志输出到命令行 */
static bool enableLogDump = false; /**< 是否允许把日志输出到文件 */
static std::map<std::string, std::shared_ptr<std::ostream>> logPath; /**< 文件路径到文件流的映射 */
static std::map<std::string, std::shared_ptr<std::ostream>> logStream; /**< 日志名称到文件流的映射 */

/**
 * @brief 开始时间计数
 *
 * @param name 标识符
 */
void startCalTime(std::string name);

/**
 * @brief 结束时间计数
 *
 * @param name 标识符，需要与开始时间对应
 * @return double 距离开始计算时间经过的毫秒数
 */
double stopCalTime(std::string name);

/**
 * @brief 设置日志文件对应的路径
 *
 * @param name 日志名称
 * @param path 路径
 */
void setLogFile(std::string name, std::string path);

/**
 * @brief 打印日志
 *
 * @param name 日志名称
 * @param message 打印的内容
 * @param outputTime 是否需要输出日志时间
 */
void printLog(std::string name, std::string message, bool outputTime = true);

/**
 * @brief 打印日志
 *
 * @param name 日志名称
 * @param message 打印的内容，数字
 */
void printLog(std::string name, long long message);

/**
 * @brief 设置是否允许把日志输出到文件
 *
 * @param enable 状态
 */
void setLogDumpStatus(bool enable);

/**
 * @brief 设置是否允许把日志输出到命令行
 *
 * @param enable 状态
 */
void setLogPrintStatus(bool enable);

/**
 * @brief 设置是否允许把时间输出到命令行
 *
 * @param enable 状态
 */
void setTimePrintStatus(bool enable);

/**
 * @brief 清空所有日志文件的状态
 *
 */
void clearLogFile();
};

#endif // LOGGER_H
