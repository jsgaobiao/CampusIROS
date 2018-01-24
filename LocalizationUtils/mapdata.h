/** @file
 *  @brief 机器人在线运行框架.
 *  @author 徐良威 (XU Liangwei)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */

#ifndef MAPDATA_H
#define MAPDATA_H

// UTILS
#include <basedata.h>

// OPENCV
#include "opencv2/opencv.hpp"

// QT
#include <QDebug>

/**
 * @brief 存储地图的基本信息
 *
 */
class MapInfo {
public:

    /**
     * @brief 默认的地图信息
     *
     * @param initialOx 起点x坐标
     * @param initialOy 起点y坐标
     * @param width 宽度，默认1000像素
     * @param height 高度，默认1000像素
     * @param pixelSize 一像素对应的真实距离，默认100mm
     * @param filename 存储的文件名，可为空
     * @param filePath 存储的文件路径，可为空
     */
    MapInfo(double initialOx = 0, double initialOy = 0, int width = 1000, int height = 1000, int pixelSize = 100,
            std::string filename = "", std::string filePath = "");
    std::pair<int, int> getIndex();

    /**
     * @brief 把地图对齐到最小单位的倍数上
     *
     * @return MapInfo 对齐后的地图信息
     */
    MapInfo normalize();

    double initialOx; /**< 起点处［像素（0，0）点］的x坐标，以m为单位 */
    double initialOy; /**< 起点处［像素（0，0）点］的y坐标，以m为单位 */
    int ox; /**< 起点处［像素（0，0）点］的x坐标，以mm为单位，舍入为整数 */
    int oy; /**< 起点处［像素（0，0）点］的y坐标，以mm为单位，舍入为整数 */

    int pixelSize; /**< 单位像素对应的实际距离，以mm为单位 */
    int unit; /**< 一个地图单元，即每一个存储在硬盘中的方块的边长（像素） */
    int gridLength; /**< 一个地图单元，即每一个存储在硬盘中的方块的边长，以mm为单位 */
    int width; /**< 该地图的宽度（像素） */
    int height; /**< 该地图的高度（像素） */
    int rangeMinX; /**< 该地图的有效部分的最小x坐标（mm） */
    int rangeMinY; /**< 该地图的有效部分的最小y坐标（mm） */
    int rangeMaxX; /**< 该地图的有效部分的最大x坐标（mm） */
    int rangeMaxY; /**< 该地图的有效部分的最大y坐标（mm） */

    std::string path; /**< 该地图的存储位置 */

    std::string filename; /**< 该地图的文件名 */

};

/**
 * @brief 实际存储地图信息的部分
 *
 */
class MapBody {
public:
    /**
     * @brief 默认的地图块
     *
     * @param initialOx 起点x坐标
     * @param initialOy 起点y坐标
     * @param width 宽度，默认1000像素
     * @param height 高度，默认1000像素
     * @param pixelSize 一像素对应的真实距离，默认100mm
     * @param isNormalized 是否已经对齐到单位格子，默认否
     * @param type 地图数据类型
     * @param filename 地图文件位置
     */
    MapBody(double initialOx = 0, double initialOy = 0, int width = 1000, int height = 1000, int pixelSize = 100, bool isNormalized = false, int type = CV_8U, std::string filename = "");

    /**
     * @brief 使用MapInfo初始化
     *
     * @param info 地图信息
     * @param isNormalized 是否已经对齐到单位格子，默认否
     * @param type 地图数据类型
     * @param filename 地图文件位置
     */
    MapBody(MapInfo info, bool isNormalized = false, int type = CV_8U, std::string filename = "");
    MapBody(const MapBody& other);

    /**
     * @brief 从硬盘中加载一块地图
     *
     * @param name 文件名
     * @return bool 是否加载成功
     */
    bool load(std::string name);

    /**
     * @brief 把一块地图存入硬盘
     *
     * @param path 文件路径
     * @return bool 是否存入成功
     */
    bool dump(std::string path);

    /**
     * @brief 复制一份地图
     *
     * @return MapBody 复制后的地图，没有共用的内存
     */
    MapBody clone();

    /**
     * @brief 重新初始化地图
     *
     */
    void recreate();

    /**
     * @brief 把大地图切分为单位大小的地图
     *
     * @return std::vector<MapBody> 地图块列表
     */
    std::vector<MapBody> split();

    /**
     * @brief 不切分地图，但获得切分为单位大小的地图信息
     *
     * @return std::map<std::pair<int, int>, MapInfo>
     */
    std::map<std::pair<int, int>, MapInfo> getSplitInfo();

    /**
     * @brief 把地图对齐到最小单位的倍数上
     *
     * @return MapBody 对齐后的地图
     */
    MapBody normalize();

    /**
     * @brief 判断地图是否为全空，如果是多通道的，则需要指定通道
     *
     * @param channel 单通道地图时小于0，否则为需要计算的通道
     * @return bool 是否为空
     */
    bool isFree(int channel = -1);

    /**
     * @brief 由地图的唯一索引生成为文件名
     *
     */
    void setFilename();

    bool isNormalized;
    MapInfo info;
    cv::Mat data;
    int type;
};

/**
 * @brief 地图数据
 *
 */
class MapData : public BaseData {
public:
    MapData();
    MapData(const MapData &other);
    std::vector<MapBody> body;
    std::string scene;
};

#endif // MAPDATA_H
