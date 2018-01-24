#include "laserlineseq.h"

LaserLineSeq::LaserLineSeq(int ln) {
    lineNum = ln;
}

void LaserLineSeq::convert2DataSeq(VelodyneRingData const &data, std::vector<std::vector<float>> &outdata, char xyzitype) {
    outdata.clear();
    for (auto c : *this) {
        vector<float> linedata;
        switch (xyzitype) {
        case 'x':
            for (int j = c.start; j != c.end + 1; j++) {
                linedata.push_back(data.points[c.lineID][j].x);
            }
            break;
        case 'y':
            for (int j = c.start; j != c.end + 1; j++) {
                linedata.push_back(data.points[c.lineID][j].y);
            }
            break;
        case 'z':
            for (int j = c.start; j != c.end + 1; j++) {
                linedata.push_back(data.points[c.lineID][j].z);
            }
            break;
        case 'i':
            for (int j = c.start; j != c.end + 1; j++) {
                linedata.push_back(data.intensity[c.lineID][j]);
            }
            break;
        default:
            break;
        }
        outdata.push_back(linedata);
    }
}

void LaserLineSeq::sort() {
    std::sort(this->begin(), this->end(), [](const LaserCut & a1, const LaserCut & a2) {
        if (a1.lineID < a2.lineID) {
            return true;
        } else if (a1.lineID == a2.lineID) {
            return a1.start < a2.start;
        } else {
            return false;
        }
    });
}
