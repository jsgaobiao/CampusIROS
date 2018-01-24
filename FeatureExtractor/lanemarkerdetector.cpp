#include "lanemarkerdetector.h"

LaneMarkerDetector::LaneMarkerDetector() {}

void LaneMarkerDetector::labelLaneMarker(VelodyneRingData &data, const LaserLineSeq &sq1) {
    detectLaneMarker(data, sq1);
    for(auto c:sqLaneMarker) {
        for(size_t i = c.start; i != c.end + 1; i++) {
            data.label[c.lineID][i].set(Label::LaneMarker);
        }
    }
}

void LaneMarkerDetector::detectLaneMarker(const VelodyneRingData &data, const LaserLineSeq &sq1) {
    float thresx = 3.0;// 2.5
    float influence = 0;
    int lag = 30;
    sqLaneMarker.clear();
    std::vector<float>sigin;
    int lastID = -1;
    std::vector<int>pointIDlist;
    std::vector<int>outsignal;
    /*
     * 将同属于一条激光线的曲线段按次序拼接为一组随机信号进行峰值检测
    */
    for (size_t i = 0; i != sq1.size(); i++) {
        if (sq1[i].lineID != lastID) {
            if (i != 0) {
                for (int k = sigin.size() - 1; k != -1; k--) {
                    if (sigin[k] == 0) {
                        sigin.erase(sigin.begin() + k);
                        pointIDlist.erase(pointIDlist.begin() + k);
                        continue;
                    }
                }
                detectSignalPeak(sigin, pointIDlist, thresx, influence, lag, outsignal);
                LaserCut tmpseq;
                tmpseq.lineID = lastID;
                for (size_t c = 0; c != outsignal.size(); c++) {
                    if (c == 0) {
                        tmpseq.start = outsignal[c];
                    } else if (outsignal[c] - outsignal[c - 1] == 1) {
                        continue;
                    } else {
                        tmpseq.end = outsignal[c - 1];
                        sqLaneMarker.push_back(tmpseq);
                        tmpseq.start = outsignal[c];
                    }
                }
                if (outsignal.size() != 0) {
                    tmpseq.end = *(outsignal.end() - 1);
                    sqLaneMarker.push_back(tmpseq);
                }
            }
            sigin.clear();
            pointIDlist.clear();
            for (int j = sq1[i].start; j != sq1[i].end + 1; j++) {
                sigin.push_back(data.intensity[sq1[i].lineID][j]);
                pointIDlist.push_back(j);
            }
            lastID = sq1[i].lineID;
        }
        else {
            for (int j = sq1[i].start; j != sq1[i].end + 1; j++) {
                sigin.push_back(data.intensity[sq1[i].lineID][j]);
                pointIDlist.push_back(j);
            }
        }
    }
    for (int k = sigin.size() - 1; k != -1; k--) {
        if (sigin[k] == 0) {
            sigin.erase(sigin.begin() + k);
            pointIDlist.erase(pointIDlist.begin() + k);
            continue;
        }
    }
    detectSignalPeak(sigin, pointIDlist, thresx, influence, lag, outsignal);
    LaserCut tmpseq;
    tmpseq.lineID = lastID;
    for (size_t c = 0; c != outsignal.size(); c++) {
        if (c == 0) {
            tmpseq.start = outsignal[c];
        } else if (outsignal[c] - outsignal[c - 1] == 1) {
            continue;
        } else {
            tmpseq.end = outsignal[c - 1];
            sqLaneMarker.push_back(tmpseq);
            tmpseq.start = outsignal[c];
        }
    }
    if (outsignal.size() != 0) {
        tmpseq.end = *(outsignal.end() - 1);
        sqLaneMarker.push_back(tmpseq);
    }
}

void LaneMarkerDetector::detectSignalPeak(std::vector<float> &insignal, std::vector<int> &pointIDlist, float thresFactor, float influence, int averwid, std::vector<int> &outsignal) {
    std::vector<int>outinx;
    if (insignal.size() < averwid * 3 + 3) {
        return;
    }
    std::vector<float>sortedinsignal = insignal;
    std::sort(sortedinsignal.begin(), sortedinsignal.end());
    int reallength = insignal.size();
    /*
     * 求取峰值基线，即峰值至少需大于集合内80%的信号值
    */
    int baseline = 0.80 * reallength;// 0.75
    int basev = sortedinsignal[ceil(baseline) + 1];
    float wstdv = MathCalc::dispersionforVec(sortedinsignal, 0, baseline - 1);
    insignal.insert(insignal.end(), insignal.begin(), insignal.begin() + 3 * averwid);
    pointIDlist.insert(pointIDlist.end(), pointIDlist.begin(), pointIDlist.begin() + 3 * averwid);
    bool jumpflag = 0;
    /*
     * 局部标准差最低值，若计算值小于该值，则使用该值补全
    */
    float dthres = wstdv;
    outsignal.clear();
    if (insignal.size() < averwid * 3 + 5) {
        return;
    }
    std::vector<float>filteredSig;
    for (int i = 0; i != averwid + 1; i++) {
        filteredSig.push_back(insignal[i]);
    }
    /*
     * 初始化滑动窗信号平均值与标准差值
    */
    std::vector<float>averFilter;
    std::vector<float>stdFilter;
    for (int i = 0; i != averwid; i++) {
        averFilter.push_back(0);
        stdFilter.push_back(0);
    }
    averFilter.push_back(MathCalc::meanforVector(insignal, 0, averwid));
    stdFilter.push_back(MathCalc::dispersionforVec(insignal, 0, averwid));
    /*
     * 逐个计算滑动窗信号平均值与标准差值
    */
    for (int i = averwid + 1; i != insignal.size(); i++) {
        if (stdFilter[i - 1] < dthres) {
            stdFilter[i - 1] = dthres;
        }
        /*
         * 以信号值大于局部信号平均值超过thresFactor倍局部标准差作为峰值检测触发信号
        */
        if (insignal[i] - averFilter[i - 1] > stdFilter[i - 1] * thresFactor) {
            /*
             * 进一步细化筛分条件
            */
            if (insignal[i] > averFilter[i - 1] * 1.25 && insignal[i] > basev + 20) {// 1.5  20
                outsignal.push_back(pointIDlist[i]);
                outinx.push_back(i + 1);
            }
            dthres = dthres * 0.5 + wstdv * 2.5 * 0.5;
            float tmpfilter;
            if (!jumpflag) {
                tmpfilter = MathCalc::meanforVector(insignal, 0, i - averwid / 2);
                tmpfilter = influence * insignal[i] + (1 - influence) * tmpfilter;
                for (int j = i - averwid / 2; j != i; j++) {
                    filteredSig[j] = tmpfilter;
                }
                filteredSig.push_back(tmpfilter);
                averFilter.push_back(MathCalc::meanforVector(filteredSig, i - averwid, i));
                stdFilter.push_back(MathCalc::dispersionforVec(filteredSig, i - averwid, i));
            }
            /*
             * 检测到峰值后应降低该峰值区域对平均值以及标准差值更新的影响
            */
            else {
                filteredSig.push_back(filteredSig[i - 1]);
                averFilter.push_back(averFilter[i - 1] / averwid * (averwid - 1) + filteredSig[i] / averwid);
                stdFilter.push_back(sqrt(((stdFilter[i - 1] * stdFilter[i - 1] * (averwid - 1) + (filteredSig[i] - averFilter[i]) * (filteredSig[i] - averFilter[i])) / averwid)));
            }
            jumpflag = 1;
        }
        else {
            dthres = dthres * 0.5 + 0.5 * wstdv;
            if (jumpflag) {
                filteredSig.push_back(insignal[i] * 0.5 + insignal[i - 1] * 0.5);// 0.4 0.6
                averFilter.push_back(MathCalc::meanforVector(filteredSig, i - averwid, i));
                stdFilter.push_back(MathCalc::dispersionforVec(filteredSig, i - averwid, i));
            }
            else {
                filteredSig.push_back(insignal[i] * 0.5 + insignal[i - 1] * 0.5);
                averFilter.push_back(averFilter[i - 1] / averwid * (averwid - 1) + filteredSig[i] / averwid);
                stdFilter.push_back(sqrt(((stdFilter[i - 1] * stdFilter[i - 1] * (averwid - 1) + (filteredSig[i] - averFilter[i]) * (filteredSig[i] - averFilter[i])) / averwid)));
            }
            jumpflag = 0;
        }
    }
    insignal.erase(insignal.end() - 3 * averwid, insignal.end());
    pointIDlist.erase(pointIDlist.end() - 3 * averwid, pointIDlist.end());
}
