/** @file
 *  @brief 离线特征提取
 *  @author 梅继林 (MEI Jilin)
 *  @date 2017.12
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */

#ifndef MAINPROCESS_H
#define MAINPROCESS_H

/** @file
 *  @brief 离线特征提取
 *  @author 梅继林 (MEI Jilin)
 *  @date 2017.12
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */
#include <QObject>
#include "frameloader.h"
#include "savefeature.h"
#include <fstream>

typedef struct {
    double		x;
    double		y;
    double		z;
} point3d;

typedef struct
{
    point3d ang;
    point3d shv;
    float northaccuracy;
    float eastaccuracy;
    float downaccuracy;
    float gpsstatus;
    long long timestamp;
}NavType;

/**
 * @brief The mainProcess class 点云特征提取接口
 */
class mainProcess : public QObject
{
    Q_OBJECT
public:
    explicit mainProcess(QObject *parent = 0);
    ~mainProcess();

    /**
     * @brief loadNavFile 读取原始Nav文件
     * @param filename 文件路径
     * @return 1-读取成功 0-读取失败
     */
    bool loadNavFile(QString filename);

    /**
     * @brief setDSSavePath 设置DS数据保存路径
     * @param path 路径
     * @param filename 文件名
     */
    void setDSSavePath(QString path, QString filename);

    /**
     * @brief frameProcess 对每一帧点云数据进行处理，包含点云运动补偿
     * @return
     */
    bool frameProcess();
    bool frameProcess(int input);

    /**
     * @brief drawResult 中间处理结果可视化
     * @param img
     * @param data
     */
    void drawResult(cv::Mat img, LaserScanData const &data);

    FrameLoader frameloader; /**< 数据读取接口 >*/
    saveFeature savefeature; /**< 特帧处理接口 >*/

    cv::Mat highVis; /**< 中间处理结果可视化 >*/
    cv::Mat midVis; /**< 中间处理结果可视化 >*/
    cv::Mat gndVis; /**< 中间处理结果可视化 >*/
    cv::Mat laneVis; /**< 中间处理结果可视化 >*/

    double progress; /**< 处理进度 >*/
    VelodyneRingData framedata; /**< 一帧数据缓存 >*/


    QString getNavfilename() const;
    void setNavfilename(const QString &value);

    /**
     * @brief fixVelodyneTimeDelay 对激光特征进行运动补偿
     * @param laser 激光特征
     * @param baseNavIndex Nav索引
     * @return
     */
    bool fixVelodyneTimeDelay(LaserScanData& laser, int &baseNavIndex);

    /**
     * @brief fixVelodyneTimeDelay 对原始点云进行运动补偿
     * @param inputdata 点云数据
     * @param lasertime 激光时间戳
     * @param basegpsindex gps时间戳
     * @return
     */
    bool fixVelodyneTimeDelay(VelodyneRingData& inputdata, long long& lasertime, long &basegpsindex);

    /**
     * @brief getIsFixVelodyneTimeDelay 获取是否进行运动补偿的状态
     * @return
     */
    bool getIsFixVelodyneTimeDelay() const;
    void setIsFixVelodyneTimeDelay(bool value);

private:
    /**
     * @brief xy2Degree 根据位置计算角度
     * @param x
     * @param y
     * @return
     */
    double xy2Degree(double x, double y);

    /**
     * @brief processLaserScan 将特征数据转为range数据
     * @param src 激光特征数据
     * @param dst range数据
     */
    void processLaserScan(const LaserScanData &src, std::vector<short> &dst);


    std::ofstream highDsFile; /**< 各层次特征文件 >*/
    std::ofstream originHighDsFile;

    std::ofstream midDsFile;
    std::ofstream gndDsFile;
    std::ofstream laneDsFile;

    std::ofstream repairedVelodyneFile;

    std::ifstream gpsFile;
    std::vector<NavType> navBuf;

    std::ofstream timestampfile;

    bool isFixVelodyneTimeDelay; /**< 是否进行运动补偿 >*/

    QString navfilename;

    long long lastLaserTime;

    bool init;  /**< 初始化状态 >*/

};

#endif // MAINPROCESS_H
