/** @file
 *  @brief 机器人在线运行框架.
 *  @author 徐良威 (XU Liangwei)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */

#ifndef TYPECONVERTER_H
#define TYPECONVERTER_H

#include <string>
#include <sstream>
#include <vector>

/**
 * @brief 处理一些简单的类型转换
 *
 * 仅使用STL
 *
 */
namespace TypeConverter {
/**
 * @brief 把数字转化为std::string
 *
 * @param src 数字
 * @return std::string 目标字符串
 */
std::string toStr(long long src);

/**
 * @brief 把c类型的字符串转为标准字符串
 *
 * @param src c字符串
 * @return std::string 目标字符串
 */
std::string toStr(const char *src);

/**
 * @brief 把字符串用分隔符分开
 *
 * @param str 字符串
 * @param sep 分隔符
 * @return std::vector<std::string> 分割后的字符串
 */
std::vector<std::string> splitStr(std::string str, char sep);
};

#endif // TYPECONVERTER_H
