#include "planefitting.h"
#include "logger.h"
PlaneFitting::PlaneFitting(const LaserLineSeq *lineseqs, const VelodyneRingData *inidata, int ln) {
    srand48(time(NULL));
    seq = lineseqs;
    data = inidata;
    /*
     * 初始化部分标志位
    */
    isRANSAC = 1;
    findFlag = false;
    linenum = ln;
}

void PlaneFitting::fitPlane(int ransacIter, double toledis, double *reference) {
    /*
     * 可进一步限制选取激光线，如限制仅使用近处激光线进行平面拟合
    */
    for(auto c : *seq) {
        if (c.lineID < linenum) {
            limitSeq.push_back(c);
        }
    }
    /*
     * 检查数据是否存在
    */
    if(!(limitSeq.size())) {
        return;
    }
    /*
     *  初始化部分参数
    */
    bool findflag = 0;
    int totalnum = 0;
    int svdIterNum = 2;
    for (auto c : limitSeq) {
        totalnum += c.end - c.start + 1;
    }
    int bestinliernum = 0;
    int maxinliernum = 0;
    int i;
    int RANSACSampleNum = 1000;
    int ptsnumthres = RANSACSampleNum * 0.6;

    int checkLength = 50;
    if(totalnum < RANSACSampleNum) {
        RANSACSampleNum = totalnum;
    }
    /*
     * 使用RANSAC初定平面
    */
    if (isRANSAC) {
        int inliernum = 0;
        std::vector<cv::Point3d>ptsForFit;
        double besterror = 100000;
        for (i = 0; i != ransacIter; i++) {
            if (!(i % checkLength) && findflag == 0 && (double)ptsnumthres / RANSACSampleNum > 0.28) {
                ptsnumthres = ptsnumthres * 0.7;
            }
            inliernum = 0;
            select3PtFromLineseq(ptsForFit, 0.4);// 限制选点位置0.4m
            double para[4];
            double error = 0;
            MathCalc::calPlane(ptsForFit, para[0], para[1], para[2], para[3]);
            if (abs(para[2] / para[0]) < 40 || abs(para[2] / para[1]) < 40) {// 限定平面位置
                i -= 1;
                continue;
            }
            int sampleCnt = 0;
            while(sampleCnt < RANSACSampleNum) {
                int k = drand48() * limitSeq.size();
                int j = drand48() * (limitSeq[k].end - limitSeq[k].start + 1) + limitSeq[k].start;
                double dis = MathCalc::DistPt2Panel(data->points[limitSeq[k].lineID][j], para[0], para[1], para[2], para[3]);
                if (dis < toledis) {
                    inliernum++;
                    error += dis;
                }
                sampleCnt++;
            }
            if (inliernum > maxinliernum) {
                maxinliernum = inliernum;
            }
            if (inliernum > ptsnumthres) {
                findflag = true;
                double meanerror = error / inliernum;
                if (meanerror < besterror) {
                    memcpy((void*)optimizedParam, (void*)para, 4 * sizeof(double));
                    besterror = meanerror;
                    bestinliernum = inliernum;
                }

            }
        }
    }

    if (!findflag) {
        bestinliernum = 0;
    } else {
        double factor = optimizedParam[3];
    }
    int svdSampleNum = 2000;
    if(totalnum < svdSampleNum) {
        svdSampleNum = totalnum;
    }
    double svdPlaneParams[4];
    if(findflag && isRANSAC) {
        memcpy((void*)svdPlaneParams, (void*)optimizedParam, sizeof(double) * 4);

    } else {
        memcpy((void*)svdPlaneParams, (void*)reference, sizeof(double) * 4);
        toledis *= 2;
    }
    int iter = 0;
    assert(svdSampleNum > 4);
    /*
     * 使用SVD获得精确平面参数
    */
    Eigen::VectorXd inlierResidual(svdSampleNum);
    Eigen::MatrixXd inlierptsxyz(svdSampleNum, 3);

    while(iter < svdIterNum) {
        inlierResidual = Eigen::VectorXd::Zero(svdSampleNum);
        inlierptsxyz = Eigen::MatrixXd::Zero(svdSampleNum, 3);
        int inx = 0;
        int cnt = 0;
        while(inx < svdSampleNum) {
            int k = drand48() * limitSeq.size();
            int j = drand48() * (limitSeq[k].end - limitSeq[k].start + 1) + limitSeq[k].start;
            double dis = 0.0;
            dis = MathCalc::DistPt2Panel(data->points[limitSeq[k].lineID][j], svdPlaneParams[0], svdPlaneParams[1], svdPlaneParams[2], svdPlaneParams[3]);

            if (dis < toledis) {
                inlierptsxyz(inx, 0) = data->points[limitSeq[k].lineID][j].x;
                inlierptsxyz(inx, 1) = data->points[limitSeq[k].lineID][j].y;
                inlierptsxyz(inx, 2) = data->points[limitSeq[k].lineID][j].z;
                inx += 1;
            }
            cnt++;
            if(cnt > svdSampleNum * 4) {
                break;
            }
        }
        /*
         * 出错情况校验
        */
        if (inx <= 4) {
            qDebug() << "planefitting:147: " << inx << optimizedParam[0] << optimizedParam[1] << optimizedParam[2] << optimizedParam[3] ;
            if(optimizedParam[1] != 0){
                if(optimizedParam[2] / optimizedParam[1]<40){
                    optimizedParam[0]=optimizedParam[1]=0.0;
                    optimizedParam[2]=1;
                    optimizedParam[3]=1.93;
                }
            }
            return;
        }
        assert(inx > 4);
        inlierResidual = inlierResidual.head(inx);
        inlierptsxyz = inlierptsxyz.topRows(inx);
        double meanx = inlierptsxyz.col(0).mean();
        double meany = inlierptsxyz.col(1).mean();
        double meanz = inlierptsxyz.col(2).mean();
        Eigen::Vector3d center(meanx, meany, meanz);
        Eigen::MatrixXd tmpM(inx, 3);
        tmpM = center.transpose().replicate(inx, 1);
        inlierptsxyz = inlierptsxyz - tmpM;
        RedSVD::RedSVD<Eigen::MatrixXd> svdForpts(inlierptsxyz);
        double tmp[4];
        tmp[0] = svdForpts.matrixV()(0, 2);
        tmp[1] = svdForpts.matrixV()(1, 2);
        tmp[2] = svdForpts.matrixV()(2, 2);
        tmp[3] = -center.adjoint() * svdForpts.matrixV().col(2);

        memcpy((void*)svdPlaneParams, (void*)tmp, 4 * sizeof(double));
        double residualNow = inlierResidual.mean() / sqrt(tmp[0] * tmp[0] +
                                                          tmp[1] * tmp[1] + tmp[2] * tmp[2]);
        toledis = toledis * 0.7;
        if(toledis < 0.05) {
            toledis = 0.05;
        }
        iter++;
    }
    memcpy((void*)optimizedParam, (void*)svdPlaneParams, 4 * sizeof(double));
    Eigen::Vector3d line(optimizedParam[0], optimizedParam[1], optimizedParam[2]);

    inlierResidual = inlierptsxyz * line;
    inlierResidual = inlierResidual.cwiseAbs();
    meanResidual = inlierResidual.mean() / sqrt(optimizedParam[0] * optimizedParam[0] +
                                                optimizedParam[1] * optimizedParam[1] + optimizedParam[2] * optimizedParam[2]);// +tmp[3];the samples has been centered
    findFlag = findflag;
    /*
     * 保证平面参数符号统一
    */
    if (optimizedParam[2] < 0) {
        optimizedParam[0] = -optimizedParam[0];
        optimizedParam[1] = -optimizedParam[1];
        optimizedParam[2] = -optimizedParam[2];
        optimizedParam[3] = -optimizedParam[3];
    }
}

void PlaneFitting::select3PtFromLineseq(std::vector<cv::Point3d> &ptsForFit, double zmaxlimit) {
    ptsForFit.clear();
    int inilaneptnum = 0;
    std::vector<int>comparelist;
    for (auto c : limitSeq) {
        inilaneptnum += c.end - c.start + 1;
        comparelist.push_back(inilaneptnum);
    }
    int index[3];
    for (int i = 0; i != 3; i++) {

        index[i] = rand() % inilaneptnum;
        if (i > 0) {
            int *re;
            re = std::find(index, index + i - 1, index[i]);
            while (*re == index[i]) {
                index[i] = rand() % inilaneptnum;
            }
        }
        int tmp = 0;
        int lastc = 0;
        for (auto c : comparelist) {
            if (c >= index[i]) {
                break;
            }
            tmp++;
            lastc = c;
        }
        int id = (limitSeq)[tmp].lineID;
        int ptidx = (limitSeq)[tmp].start + index[i] - lastc;
        CvPoint3D32f tmppt;
        tmppt.x = data->points[id][ptidx].x;
        tmppt.y = data->points[id][ptidx].y;
        tmppt.z = data->points[id][ptidx].z;
        if (tmppt.z > zmaxlimit) {
            i -= 1;
            continue;
        }
        ptsForFit.push_back(tmppt);
    }
}
