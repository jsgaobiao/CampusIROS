#include "laserscandata.h"

LaserScanData::LaserScanData() {
    /*
     * 初始化各统计变量
     */
    rangeMax = 0;
    num = 0;
    validNum = 0;
    srand48(time(NULL));
}

void LaserScanData::clear() {
    /*
     * 清空所有数据，初始化各统计变量
     */
    distances.clear();
    points.clear();
    pointInx.clear();
    label.clear();
    rangeMax = 0;
    num = 0;
    validNum = 0;
}

void LaserScanData::calValidPtNum()
{
    /*
     * 距离值等于0则认为是无效点
     */
    int cnt = 0;
    for(auto dis : this->distances)
        if(dis>1e-6)
            cnt++;
    this->validNum = cnt;
}

void LaserScanData::eraseInvalidPts()
{
    /*
     * 删除无效点相关的所有数据
     */
    int q = 0;
    for(int i = 0; i < this->distances.size(); i++){
        if(distances[i] > 1e-6){
            distances[q] = distances[i];
            pointInx[q] = pointInx[i];
            points[q] = points[i];
            label[q] = label[i];
            q++;
        }
    }
    this->num = this->validNum = q;
    this->distances.resize(q);
    this->pointInx.resize(q);
    this->points.resize(q);
    this->label.resize(q);
}

void LaserScanData::sampleSelf(int num)
{
    /*
     * 采样前筛除无效点，若采样数量大于有效点数则完全采样，使用drand48()作为采样随机机制
     */
    this->eraseInvalidPts();
    double ratio = (double)num / this->validNum;
    if(ratio > 1.0)
        ratio = 1.0;
    int q = 0;
    for(int i = 0; i < this->distances.size(); i++){
        if(drand48() < ratio){
            distances[q] = distances[i];
            pointInx[q] = pointInx[i];
            points[q] = points[i];
            label[q] = label[i];
            q++;
        }
    }
    this->num = this->validNum = q;
    this->distances.resize(q);
    this->pointInx.resize(q);
    this->points.resize(q);
    this->label.resize(q);
}
