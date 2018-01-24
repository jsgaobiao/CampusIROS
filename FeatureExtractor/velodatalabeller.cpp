#include "velodatalabeller.h"

VeloFeatureExtractor::VeloFeatureExtractor() {
    /*
     * 设置默认参数
     */
    this->highFeatureRange = std::make_pair(2.0, 2.2);
    this->midFeatureRange = std::make_pair(1.0, 1.2);
    this->highRangeMax = 30.0;
    this->midRangeMax = 30.0;
    this->gndRangeMax = 30.0;
    this->setInvalidScope(4.2,1.8,2.0,2.0);
}

void VeloFeatureExtractor::setCalibInfo(VeloCalib &calibInfo) {
    this->calibInfo = &calibInfo;
}

void VeloFeatureExtractor::setRangeMax(double highRangeMax, double midRangeMax, double gndRangeMax) {
    this->highRangeMax = highRangeMax;
    this->midRangeMax = midRangeMax;
    this->gndRangeMax = gndRangeMax;
}

void VeloFeatureExtractor::setInvalidScope(double front, double rear, double left, double right) {
    this->offsetFront = front;
    this->offsetRear = rear;
    this->offsetLeft = left;
    this->offsetRight = right;
}

void VeloFeatureExtractor::processCurrentFrame(VelodyneRingData &data) {
    /*
     * 提取地面特征
     */
    groundDetector.labelGnd(data);
    /*
     * 提取并生成中、高层特征
     */
    bool baseonGnd = true;
    this->extractScanFeature(data, this->highFeature, this->highFeatureRange, this->highRangeMax, Label::HighLayerFeature, baseonGnd);
    this->extractScanFeature(data, this->midFeature, this->midFeatureRange, this->midRangeMax, Label::MidLayerFeature, baseonGnd);
    /*
     * 提取地面标志特征
     */
    laneMarkerDetector.labelLaneMarker(data, groundDetector.sqgnd);
    /*
     * 生成地面层特征数据
     */
    this->extractScanFeature(data, this->laneFeature);

    this->extractScanFeatureByLabel(data, this->gndFeature, Label::Ground, this->gndRangeMax);

}

void VeloFeatureExtractor::extractScanFeature(VelodyneRingData &framedata, LaserScanData &OutputLMS, std::pair<double, double> zRange, double rangMax, Label label, bool basePlane) {
    int lineNum = framedata.points.size();
    int lineptNum = 0;
    if(lineNum) {
        lineptNum = framedata.points[0].size();
    }
    double degreeLSB = 360.0 / lineptNum;

    OutputLMS.clear();
    OutputLMS.setTimeStamp(framedata.getTimeStamp());
    OutputLMS.num = lineptNum;
    OutputLMS.rangeMax = rangMax;// meter
    OutputLMS.initialPointNum = lineptNum;
    /*
     * 初始化中、高层特征数据
     */
    for (size_t i = 0; i != OutputLMS.num; i++) {
        OutputLMS.distances.push_back(0.0);
        OutputLMS.points.push_back(std::make_pair(0.0, 0.0));
        OutputLMS.pointInx.push_back(std::make_pair(-1, -1));
        OutputLMS.label.push_back(Label::Unknown);
    }
    /*
     * 提取中、高层特征，考虑激光线首点角度偏移，读取激光标定文件以提取在设定高度范围内同一角度上距车体最近的激光点
     */
    #pragma omp parallel for
    for(size_t i = 0; i != lineNum; i++) {
        int inxbias = this->calibInfo->rotCorrection[this->calibInfo->idx[i]] / degreeLSB;
        if(!framedata.points[i].size()) {
            continue;
        }
        for(size_t j = 0; j != lineptNum; j++) {
            int newPtInx = (j + inxbias + lineptNum) % lineptNum;
            double zValue;
            if(basePlane) {
                zValue = MathCalc::DistPt2Panel(framedata.points[i][newPtInx], this->groundDetector.gndPlaneParams[0], this->groundDetector.gndPlaneParams[1], this->groundDetector.gndPlaneParams[2], this->groundDetector.gndPlaneParams[3]);
            } else {
                zValue = framedata.points[i][newPtInx].z;
            }
            if (zValue > zRange.first && zValue <= zRange.second) {
                double tmpdis = (sqrt(framedata.points[i][newPtInx].x * framedata.points[i][newPtInx].x + framedata.points[i][newPtInx].y * framedata.points[i][newPtInx].y));
                double lmsx = framedata.points[i][newPtInx].x;
                double lmsy = framedata.points[i][newPtInx].y;
                /*
                 * 去除车体包围框内的点，防止击中主体车的激光点造成干扰
                 */
                if(lmsy < offsetFront && lmsy > -offsetRear && lmsx < offsetLeft && lmsx > -offsetRight) {
                    continue;
                }
                if(tmpdis > OutputLMS.rangeMax) {
                    continue;
                }
                if (OutputLMS.distances[lineptNum - newPtInx - 1] < 1e-6 || OutputLMS.distances[lineptNum - newPtInx - 1] > tmpdis) {
                    OutputLMS.distances[lineptNum - newPtInx - 1] = tmpdis;
                    OutputLMS.points[lineptNum - newPtInx - 1] = std::make_pair(lmsx, lmsy);
                    OutputLMS.pointInx[lineptNum - newPtInx - 1] = std::make_pair(i, newPtInx);
                    framedata.label[i][newPtInx].set(label);
                    OutputLMS.label[lineptNum - newPtInx - 1] = framedata.label[i][newPtInx];
                }
            }
        }
    }
    OutputLMS.calValidPtNum();
}

void VeloFeatureExtractor::extractScanFeature(VelodyneRingData &framedata, LaserScanData &groundFeature) {
    groundFeature.clear();
    groundFeature.rangeMax = this->gndRangeMax;
    if(framedata.points.size()) {
        groundFeature.initialPointNum = framedata.points[0].size();
    }
    groundFeature.setTimeStamp(framedata.getTimeStamp());
    #pragma omp parallel for
    for(int i = 0; i < framedata.label.size(); i++) {
        for(int j = 0; j != framedata.label[i].size(); j++) {
            if(framedata.label[i][j].is(Label::LaneMarker) && framedata.distance[i][j] < groundFeature.rangeMax) {
                groundFeature.distances.push_back(framedata.distance[i][j]);
                groundFeature.pointInx.push_back(std::make_pair(i, j));
                groundFeature.points.push_back(std::make_pair(framedata.points[i][j].x, framedata.points[i][j].y));
                framedata.label[i][j].set(Label::Ground);
                groundFeature.label.push_back(framedata.label[i][j]);
            }
        }
    }
    groundFeature.num = groundFeature.validNum = groundFeature.points.size();
}

//void VeloFeatureExtractor::pruneInvalidPoint(VelodyneRingData &data, NavData locPos, double localizationBeliefThres, const MapData &mapData) {
//    movObjLabeller.labelStrangeObject(locPos, localizationBeliefThres, data, this->highFeature, this->midFeature, mapData);
//    /*
//     * 执行删除无效点操作（与地图严重不匹配的障碍物，多为动态障碍物）
//     */
//    if(locPos.confidence > localizationBeliefThres) {
//        movObjLabeller.pruneInvalidPoint(this->highFeature);
//        movObjLabeller.pruneInvalidPoint(this->midFeature);
//    }
//}
void VeloFeatureExtractor::extractScanFeatureByLabel(const VelodyneRingData &framedata, LaserScanData &Feature, Label label, double rangeMax)
{
    Feature.clear();
    Feature.setTimeStamp(framedata.getTimeStamp());
    for(int i = 0; i != framedata.label.size(); i++) {
        for(int j = 0; j != framedata.label[i].size(); j++) {
            if(framedata.label[i][j].is(label) && framedata.distance[i][j] < rangeMax) {
                Feature.distances.push_back(framedata.distance[i][j]);
                Feature.pointInx.push_back(std::make_pair(i, j));
                Feature.points.push_back(std::make_pair(framedata.points[i][j].x, framedata.points[i][j].y));
                Feature.label.push_back(framedata.label[i][j]);
            }
        }
    }
    // gnd feature num assignment
    Feature.num = Feature.validNum = Feature.points.size();
}
