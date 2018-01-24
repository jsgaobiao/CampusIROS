/** @file
 *  @brief 机器人在线运行框架.
 *  @author 徐良威 (XU Liangwei)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */

#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <vector>
#include <string>
#include <map>
#include <eigen3/Eigen/Dense>

/**
 * @brief 用于管理坐标变换的各种函数
 *
 */
namespace Transform {

class TransformInfo;

/**
 * @brief 二维点
 *
 */
class Point2 {
public:
    Point2(double x, double y) : x(x), y(y), valid(true) {}
    Point2(bool valid = true) : valid(valid) {}
    double x;
    double y;
    bool valid;  /**< 是否为合法点 */
};

/**
 * @brief 三维点
 *
 */
class Point3 : public Point2 {
public:
    Point3(bool valid = true) : Point2(valid) {}
    Point3(double x, double y, double z) : Point2(x, y), z(z) {}

    /**
     * @brief 旋转点
     *
     * @param info 旋转参数
     */
    void rotate(TransformInfo info);

    /**
     * @brief 平移点
     *
     * @param info 平移参数
     */
    void shift(TransformInfo info);
    double z;
};

/**
 * @brief 把一个点进行绕原点的旋转变换
 *
 * @param p 三维点
 * @param yaw yaw方向的旋转，rad
 * @param pitch pitch方向的旋转，rad
 * @param roll roll方向的旋转，rad
 * @return Point3 旋转后的点
 */
Point3 rotatePoint(Point3 p, double yaw, double pitch = 0, double roll = 0);

/**
 * @brief 把一个点进行绕原点的旋转变换
 *
 * @param p 三维点
 * @param mat 三维旋转矩阵
 * @return Point3 旋转后的点
 */
Point3 rotatePoint(Point3 p, Eigen::Matrix3d mat);

/**
 * @brief 把一个点进行平移变换
 *
 * @param p 三维点
 * @param x 平移向量的x
 * @param y 平移向量的y
 * @param z 平移向量的z
 * @return Point3 平移后的点
 */
Point3 shiftPoint(Point3 p, double x, double y, double z);

/**
 * @brief 把一个点进行平移变换
 *
 * @param p 三维点
 * @param trans 平移矩阵
 * @return Point3 平移后的点
 */
Point3 shiftPoint(Point3 p, Eigen::Vector3d trans);

/**
 * @brief 把一个点进行三维变换，先平移，后旋转
 *
 * @param p 三维点
 * @param rot 旋转矩阵
 * @param trans 平移矩阵
 * @return Point3 变换后的点
 */
Point3 transformPoint(Point3 p, Eigen::Matrix3d rot, Eigen::Vector3d trans);

/**
 * @brief 坐标变换关系
 *
 * 前向 x，左向 y，与地面垂直 z，单位m \n
 * 绕 z 旋转 yaw， 绕 y 旋转 pitch，绕 x 旋转 roll， 单位rad，逆时针为正
 *
 */
class TransformInfo {
public:
    TransformInfo(bool valid = true);
    /**
     * @brief
     *
     * @param x
     * @param y
     * @param z
     * @param yaw
     * @param pitch
     * @param roll
     * @param valid
     */
    TransformInfo(double x, double y, double z, double yaw = 0, double pitch = 0, double roll = 0, bool valid = true);

    /**
     * @brief
     *
     * @param rot
     * @param shift
     * @param valid
     */
    TransformInfo(Eigen::Matrix3d rot, Eigen::Vector3d shift, bool valid = true);
    Eigen::Matrix3d rot; /**< 旋转矩阵 */
    Eigen::Vector3d shift; /**< 平移向量 */
    bool valid; /**< 是否合法 */
    double eps = 1e-6;  /**< 用于比较两个浮点数相等 */

    /**
     * @brief 判断两个变换关系是否相等
     *
     * @return bool 旋转和平移都相等为相等
     */
    bool operator==(const TransformInfo& other);

    /**
     * @brief 当前变换的逆变换
     *
     * @return TransformInfo 逆变换
     */
    TransformInfo operator-();

    /**
     * @brief 合成两个坐标转换
     *
     * @return TransformInfo 合成后的坐标转换关系
     */
    TransformInfo operator+(const TransformInfo& other);
};

static std::vector<std::string> frameList;
static std::map<std::pair<std::string, std::string>, TransformInfo> infoList;

/**
 * @brief 添加一个坐标系变换
 *
 * @param info 变换关系
 * @param src 起始坐标系
 * @param dst 目标坐标系
 * @return bool 是否添加成功，如果与已存在的变换矛盾，返回false
 */
bool addInfo(TransformInfo info, std::string src, std::string dst);

/**
 * @brief 获得src中坐标为p的点在dst中的坐标
 *
 * @param p 空间坐标点
 * @param src 起始坐标系
 * @param dst 目标坐标系
 * @return Point3 在目标坐标系中的坐标点，如果变换不存在，返回的点valid值为false
 */
Point3 getPoint(Point3 p, std::string src, std::string dst);

/**
 * @brief 获得src->dst的变换
 *
 * @param src 起始坐标系
 * @param dst 目标坐标系
 * @return TransformInfo 变换关系，如果不存在，返回的变换valid值为false
 */
TransformInfo getInfo(std::string src, std::string dst);

/**
 * @brief 清除已有的变换关系
 *
 */
void clear();

/**
 * @brief 更新和src、dst相关的所有frame
 *
 * @param src 起始坐标系
 * @param dst 目标坐标系
 * @return bool 是否更新成功
 */
bool update(std::string src, std::string dst);

/**
 * @brief 判断是否已经存在某个坐标系
 *
 * @param frame 坐标系名称
 * @return bool 是否存在
 */
bool existFrame(std::string frame);

/**
 * @brief 判断是否存在某个src->dst的变换
 *
 * @param src 起始坐标系
 * @param dst 目标坐标系
 * @return bool 是否存在
 */
bool existLink(std::string src, std::string dst);
}

#endif // TRANSFORM_H
