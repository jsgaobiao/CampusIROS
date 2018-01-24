#include "velodyneringdata.h"

VelodyneRingData::VelodyneRingData(size_t lineNum, int maxLinePointNum)
    : lineNum(lineNum), enable(lineNum), points(lineNum), distance(lineNum), angle(lineNum),
    intensity(lineNum), label(lineNum), maxLinePointNum(maxLinePointNum) {}

void VelodyneRingData::clear() {
    for (int i = 0; i < points.size(); i++) points[i].clear();
    for (int i = 0; i < distance.size(); i++) distance[i].clear();
    for (int i = 0; i < angle.size(); i++) angle[i].clear();
    for (int i = 0; i < intensity.size(); i++) intensity[i].clear();
    for (int i = 0; i < label.size(); i++) label[i].clear();
    for (int i = 0; i < enable.size(); i++) enable[i].clear();
}

double VelodyneRingData::calPointsDistance(unsigned int lineID1, unsigned int pointIndex1, unsigned int lineID2, unsigned int pointIndex2) {
    if (lineID1 > this->points.size() - 1 || lineID2 > this->points.size() - 1 || pointIndex1 > this->points[lineID1].size() - 1 || pointIndex2 > this->points[lineID2].size() - 1) {
        return -1;
    }
    double x1 = this->points[lineID1][pointIndex1].x;
    double y1 = this->points[lineID1][pointIndex1].y;
    double z1 = this->points[lineID1][pointIndex1].z;
    double x2 = this->points[lineID2][pointIndex2].x;
    double y2 = this->points[lineID2][pointIndex2].y;
    double z2 = this->points[lineID2][pointIndex2].z;
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
}

int VelodyneRingData::getMaxLinePointNum() const {
    return maxLinePointNum;
}

int VelodyneRingData::getLineNum() const {
    return lineNum;
}
