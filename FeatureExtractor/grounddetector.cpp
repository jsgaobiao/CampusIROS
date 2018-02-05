#include "grounddetector.h"
#include <QDebug>
GroundDetector::GroundDetector(int ln) : sq1(ln), sqgnd(ln) {
    linenum = ln;
    double scale;
    /*
     * 初始化相对于激光雷达的基本地平面参数
    */
    gndPlaneParams[0] = 0;
    gndPlaneParams[1] = 0;
    gndPlaneParams[2] = 1;
    gndPlaneParams[3] = 1.93;
    /*
     * 初始化激光雷达各线平滑度阈值
    */
    double inivar = 0.0015;
    for(int i = 0; i != linenum; i++) {
        if (i < 6) {
            scale = 0.3;
        } else if(i < int(linenum / 3.0)) {     // 20
            scale = 0.5;
        } else if (i < int(linenum / 2.0)) {    // 30
            scale = 1.5;
        } else if (i < int(linenum * 0.66)) {   // 40
            scale = 3.0;
        } else if (i < int(linenum * 0.83)) {   // 50
            scale = 5.0;
        } else {
            scale = 8.0;
        }
        varThres[i] = scale * inivar;
    }
}

void GroundDetector::lineSlideSmooth(const std::vector<cv::Point3d> &linein, std::vector<cv::Point3d> &lineout, int num) {
    std::vector<cv::Point3d> lineouttmp;
    for (int i = 0; i != linein.size(); i++) {
        cv::Point3d inipoint;
        inipoint.x = 0; inipoint.y = 0; inipoint.z = 0;
        lineouttmp.push_back(inipoint);
        if (linein[i].x == 0 || linein[i].y == 0 ) {
            continue;
        }
        float sumx = 0, sumy = 0, sumz = 0;
        int invalidnum = 0;
        for (int j = 0; j != num; j++) {
            int index = (i - num / 2 + j + linein.size()) % linein.size();
            if (linein[index].x == 0 || linein[index].y == 0) {
                invalidnum++;
                continue;
            }
            sumx += linein[index].x;
            sumy += linein[index].y;
            sumz += linein[index].z;
        }
        int validnum = num;
        if (invalidnum) {
            validnum -= invalidnum;
        }
        lineouttmp[i].x = sumx / validnum;
        lineouttmp[i].y = sumy / validnum;
        lineouttmp[i].z = sumz / validnum;
    }
    lineout = lineouttmp;
}

void GroundDetector::labelSmoothSeqForScans(VelodyneRingData const &data, int minpoint, int varwid) {
    if(sq1.size()) {
        sq1.clear();
    }
    LaserLineSeq sq;
    int lineInxLimit = data.getLineNum();
    if(data.points.size() < lineInxLimit) {
        lineInxLimit = data.points.size();
    }
    /*
     * 按照激光线角度分配方差阈值尺度，距离越远，阈值越高
    */
    for (int i = 0; i != lineInxLimit; i++) {
        std::vector<int> tmpseq;
        std::vector<double>window_var(data.points[i].size());
        std::vector<double>validPtDis;
        std::vector<int>validPtInx;
        int endj = 0;
        bool endflag = 0;
        for (int j = 0; j != data.points[i].size(); j++) {
            if(data.enable[i][j]) {
                validPtDis.push_back(data.distance[i][j]);
                validPtInx.push_back(j);
            }
        }
        double sumdis_tmp;
        double averdis;
        /*
         * 计算局部方差平滑度
        */
        if(validPtDis.size()) {
            sumdis_tmp = 0;
            for (int k = 0; k != varwid; k++) {
                int index = 0 - varwid / 2 + k;
                if(index < 0) {
                    index += validPtDis.size();
                }
                sumdis_tmp += validPtDis[index];
            }
            averdis = sumdis_tmp / varwid;
            window_var[validPtInx[0]] = 0;
            for (int k = 0; k != varwid; k++) {
                int index = 0 - varwid / 2 + k;
                if(index < 0) {
                    index += validPtDis.size();
                }
                window_var[validPtInx[0]] += MathCalc::sqr(validPtDis[index] - averdis);
            }
            window_var[validPtInx[0]] /= varwid;
        }
        for(int j = 1; j < validPtInx.size(); j++) {
            int pruneInx;
            int newInx;
            pruneInx = j - varwid / 2 - 1;
            newInx = pruneInx + varwid;
            if(pruneInx < 0) {
                pruneInx += validPtInx.size();
            }
            if(newInx > validPtInx.size() - 1) {
                newInx -= validPtInx.size();
            }
            sumdis_tmp = sumdis_tmp - validPtDis[pruneInx] + validPtDis[newInx];
            double averdis_n = sumdis_tmp / varwid;
            window_var[validPtInx[j]] =
                window_var[validPtInx[j - 1]] +
                (MathCalc::sqr(validPtDis[newInx]) - MathCalc::sqr(validPtDis[pruneInx])) / varwid +
                (averdis + averdis_n) * (averdis - averdis_n);
            averdis = averdis_n;
        }
        /*
         * 按照阈值筛分平滑点
        */
        for (int j = 0; j != data.points[i].size(); j++) {
            double adjustvarthres = varThres[i];
            if (window_var[j]  < adjustvarthres && window_var[j] > 0) {
                endflag = 0;
                if (tmpseq.size() == 0) {
                    tmpseq.push_back(j);
                    endj = j;
                } else if (window_var[j - 1]  < adjustvarthres && window_var[j - 1] > 0) {
                    endj = j;
                } else {
                    tmpseq.push_back(j);
                    endj = j;
                }

            } else {
                if (tmpseq.size() != 0 && endflag == 0) {
                    tmpseq.push_back(endj);
                    endflag = 1;
                }

            }

        }
        if (endj == data.points[i].size() - 1) {
            tmpseq.push_back(endj);
        }

        if (tmpseq.size() < 2) {
            continue;
        }

        for (int q = 0; q < tmpseq.size(); q += 2) {
            if (tmpseq[q + 1] - tmpseq[q] + 1 >= minpoint) {
                LaserCut tmp;
                tmp.lineID = i;
                tmp.start = tmpseq[q];
                tmp.end = tmpseq[q + 1];
                tmp.disvar = 0;
                for(size_t im = tmp.start; im <= tmp.end; im++) {
                    tmp.disvar += window_var[im];
                }
                tmp.disvar /= (tmp.end - tmp.start + 1);
                sq.push_back(tmp);
            }
        }
        if (tmpseq.size() >= 4) {
            if (tmpseq[tmpseq.size() - 1] == data.points[i].size() - 1 && tmpseq[0] == 0) {
                if (tmpseq[1] + data.points[i].size() - tmpseq[tmpseq.size() - 2] + 1 >= minpoint) {
                    LaserCut tmp;
                    tmp.lineID = i;
                    tmp.start = 0;
                    tmp.end = tmpseq[1];
                    tmp.disvar = 0;
                    for(size_t im = tmp.start; im <= tmp.end; im++) {
                        tmp.disvar += window_var[im];
                    }
                    tmp.disvar /= (tmp.end - tmp.start + 1);
                    if (tmp.end + 1 < minpoint) {
                        sq.push_back(tmp);
                    }
                    tmp.start = tmpseq[tmpseq.size() - 2];
                    tmp.end = data.points[i].size() - 1;
                    tmp.disvar = 0;
                    for(size_t im = tmp.start; im <= tmp.end; im++) {
                        tmp.disvar += window_var[im];
                    }
                    tmp.disvar /= (tmp.end - tmp.start + 1);
                    if (tmp.end - tmp.start + 1 < minpoint) {
                        sq.push_back(tmp);
                    }
                }
            }
        }
    }
    /*
     * 可设置corrodenum做平滑线段的局部腐蚀
    */
    int corrodenum = 0;
    for (auto &c : sq) {
        int invalidnum = 0;
        for (int index = c.start; index != c.end; index++) {
            if (data.points[c.lineID][index].x == 0) {
                invalidnum++;
            }
        }
        if (invalidnum > 0.7 * (c.end - c.start)) {
            c.valid = 0;
        } else {

            c.valid = 1;
            if (c.start != 0) {
                c.start += corrodenum;
            }
            if (c.start > c.end) {
                continue;
            }
            if (c.end != data.points[c.lineID].size()) {
                c.end -= corrodenum;
            }
            if (c.end < c.start) {
                continue;
            }
            sq1.push_back(c);

        }

    }

    /*
     * 补偿填充由于小幅度干扰造成的平滑曲线段的较小间隙
    */
    int gap[64] = {
        10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
        9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
        8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
        6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
        6, 6, 6, 6, 6, 5, 5, 5, 5, 5,
        5, 5, 5, 5
    };
    std::sort(sq1.begin(), sq1.end(), [](const LaserCut& a1, const LaserCut& a2) {
        if(a1.lineID < a2.lineID) {
            return true;
        } else if(a1.lineID == a2.lineID) {
            return a1.start < a2.start;
        } else {
            return false;
        }
    });
    std::vector<int>eraselist(sq1.size());
    if(sq1.size()) {
        for(size_t sqinx = sq1.size() - 1; sqinx != 0; sqinx--) {
            if(sq1[sqinx].lineID == sq1[sqinx - 1].lineID && sq1[sqinx].start - sq1[sqinx - 1].end <= gap[sq1[sqinx].lineID] + 2 * corrodenum) {
                sq1[sqinx - 1].end = sq1[sqinx].end;
                eraselist[sqinx] = 1;
            }
        }
    } else {
        qDebug() << "Error to be fixed: sq1.size()==0, timeStamp:" << data.getTimeStamp().getTickCount() << "dataSize == "
                 << data.points.size() << (data.points.size() > 0 ? data.points[0].size() : 0);
    }
    LaserLineSeq sq2 = sq1;
    sq1.clear();
    for(size_t i = 0; i != sq2.size(); i++) {
        if(eraselist[i] != 1) {
            sq1.push_back(sq2[i]);
        }
    }

}

void GroundDetector::fitGndPlane(VelodyneRingData const& data) {
    /*
     * 调用平面拟合类进行地面参数计算
    */
    planefit = PlaneFitting(&sq1, &data, data.getLineNum());
    int ransacIter = 50;
    double toleDis = 0.08;

    planefit.fitPlane(ransacIter, toleDis, gndPlaneParams);

    if (planefit.findFlag && planefit.meanResidual < 0.01) {
        memcpy((void*)gndPlaneParams, (void*)planefit.optimizedParam, 4 * sizeof(double));
        gndPlaneRes = planefit.meanResidual;
    } else if (planefit.findFlag && planefit.meanResidual < 0.05) {
        /*
         * 在拟合平均残差不优时做惯性滤波
        */
        double p1, p2;
        p1 = 0.7;
        p2 = 1 - p1;
        for (int i = 0; i != 4; i++) {
            gndPlaneParams[i] = gndPlaneParams[i] * p1 + planefit.optimizedParam[i] * p2;
        }
        gndPlaneRes = gndPlaneRes * p1 + planefit.meanResidual * p2;
    }
}

void GroundDetector::labelGnd(VelodyneRingData &data) {
    sqgnd.clear();
    labelSmoothSeqForScans(data, 20, 15);
    fitGndPlane(data);
    /*
     * 计算平滑曲线段距离地平面的高度，筛除非地面曲线段，标记地面曲线段上的激光点为地面点
    */
    for (int i = 0; i < data.points.size(); i ++) {
        for (int j = 0; j < data.points[i].size(); j ++) {
            double tmp = MathCalc::DistPt2Panel(data.points[i][j], gndPlaneParams[0], gndPlaneParams[1], gndPlaneParams[2], gndPlaneParams[3]);
            if (tmp < 0.05) {
                data.label[i][j].set(Label::Ground);
            }
        }
    }
    for (auto &c : sq1) {
        double tmpSumDis = 0;
        for (size_t inx = c.start; inx <= c.end; inx++) {
            tmpSumDis += MathCalc::DistPt2Panel(data.points[c.lineID][inx], gndPlaneParams[0], gndPlaneParams[1], gndPlaneParams[2], gndPlaneParams[3]);
        }
        c.averH = tmpSumDis / (c.end - c.start + 1);

        if (c.lineID < int(data.getLineNum() / 6.0)) {
            if (c.averH < 0.3 && c.disvar < 8e-5) {   // 10
                c.label = Label::Ground;
            }
        }
        else
        if (c.lineID < int(data.getLineNum() / 4.0)) {
            if (c.averH < 0.3 && c.disvar < 3e-4) {   // 16
                c.label = Label::Ground;
            }
        }
        else
        if (c.lineID < int(data.getLineNum()) / 2.0) {
            if (c.averH < 0.3 && c.disvar < 6e-4) {  // 32
                c.label = Label::Ground;
            }
        }
        else
        if (c.lineID < int(data.getLineNum())) {
            if (c.averH < 0.3 && c.disvar < 3e-3) { // 48
                c.label = Label::Ground;
            }
        }
    }
    for(auto c:sq1) {
        if(c.label == Label::Ground) {
            sqgnd.push_back(c);
            for(size_t i = c.start; i != c.end + 1; i++) {
                data.label[c.lineID][i].set(Label::Ground);
            }
        }
    }
}
