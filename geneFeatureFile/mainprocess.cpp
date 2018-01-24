#include "mainprocess.h"
#include <QDebug>
#include <QTime>
#include <QDir>
#include <transform.h>

mainProcess::mainProcess(QObject *parent) :
    QObject(parent)
{
    this->highVis.create(200,200,CV_8UC3);
    this->midVis.create(200,200,CV_8UC3);
    this->gndVis.create(200,200,CV_8UC3);
    this->laneVis.create(200,200,CV_8UC3);

    Transform::TransformInfo info(-1.2, 0, 0, -M_PI_2, 0, 0);
    Transform::addInfo(info, "Laser", "GPS");

    progress = 0;
    init = false;
}

mainProcess::~mainProcess()
{
    highDsFile.close();
    midDsFile.close();
    gpsFile.close();
}

bool mainProcess::loadNavFile(QString filename)
{
    gpsFile.open(filename.toStdString().c_str());
    if (!gpsFile.is_open()){
        qDebug()<<" load nav error";
        return 0;
    }

    NavType oneNav;
    int tmp;
    bool isfirst = 0;
    NavType firstdata;
    while (!gpsFile.eof())
    {
        if (!isfirst)
        {
            isfirst = 1;
            gpsFile>>firstdata.timestamp>>firstdata.ang.x>>firstdata.ang.y>>firstdata.ang.z>>firstdata.shv.x>>firstdata.shv.y
                    >>firstdata.shv.z>>firstdata.northaccuracy>>firstdata.eastaccuracy>>firstdata.downaccuracy>>firstdata.gpsstatus;
        }
        else
        {
            gpsFile>>oneNav.timestamp>>oneNav.ang.x>>oneNav.ang.y>>oneNav.ang.z>>oneNav.shv.x>>oneNav.shv.y
                    >>oneNav.shv.z>>oneNav.northaccuracy>>oneNav.eastaccuracy>>oneNav.downaccuracy>>oneNav.gpsstatus;

            //oneNav.ang.z = oneNav.ang.z - firstdata.ang.z;
            navBuf.push_back(oneNav);
        }
    }
    return 1;
}

void mainProcess::setDSSavePath(QString path, QString filename)
{
    float angrng = 360;
    float angres = 360 / 1799.0;
    float unit = 100;
    QString timestring = QDateTime::currentDateTime().toString("yyyyMMddhhmmss");
    path += "/";

    QString highdsfilename = path + filename + "_high.ds";
    highDsFile.open(highdsfilename.toStdString().c_str(), std::ios_base::binary);
    highDsFile.write((char*)&angrng, sizeof(float));
    highDsFile.write((char*)&angres, sizeof(float));
    highDsFile.write((char*)&unit, sizeof(float));

//    QString originhighdsfilename = path + "high_origin.ds";
//    originHighDsFile.open(originhighdsfilename.toStdString().c_str(), std::ios_base::binary);
//    originHighDsFile.write((char*)&angrng, sizeof(float));
//    originHighDsFile.write((char*)&angres, sizeof(float));
//    originHighDsFile.write((char*)&unit, sizeof(float));

    QString middsfilename = path + filename + "_mid.ds";
    midDsFile.open(middsfilename.toStdString().c_str(), std::ios_base::binary);
    midDsFile.write((char*)&angrng, sizeof(float));
    midDsFile.write((char*)&angres, sizeof(float));
    midDsFile.write((char*)&unit, sizeof(float));

    QString gnddsfilename = path + filename + "_gnd.xy";
    gndDsFile.open(gnddsfilename.toStdString().c_str(), std::ios_base::binary);

    QString lanedsfilename = path + filename + "_lane.xy";
    laneDsFile.open(lanedsfilename.toStdString().c_str(), std::ios_base::binary);

    QString timestampfilename =path + filename + "_timediff.txt";
    this->timestampfile.open(timestampfilename.toStdString().c_str());
    timestampfile<<"NavTime\tLaserTime\tdiff\n";
}

double mainProcess::xy2Degree(double x, double y)
{
    double ret = 0;
    if (x > 0 && y < 0 ) {
        ret = atan(-y / x);
    } else if( x < 0 && y < 0 ) {
        ret = atan(x / y) + M_PI / 2.0;
    } else if( x < 0 && y > 0 ) {
        ret = atan(y / (-x)) + M_PI;
    } else if(x > 0 && y > 0) {
        ret = atan(x / y) + M_PI * 1.5;
    }
    ret = (ret / M_PI * 180.0);

    return 360 - ret;
}

void mainProcess::processLaserScan(const LaserScanData &src, std::vector<short> &dst)
{
    dst.clear();
    dst.resize(1800);
    for (int i = 0; i < dst.size(); i++) {
        dst[i] = 32760;
    }
    for (int i = 0; i < src.points.size(); i++) {
        double x = src.points[i].first;
        double y = src.points[i].second;
        double dis = sqrt(x * x + y * y)*100.0;
        double degree = xy2Degree(x, y);

        int remapDegree = int(degree / 0.2);
        if (remapDegree >= 1800) {
            remapDegree = 1799;
        }
        if (dst[remapDegree] > dis) {
            dst[remapDegree] = short(dis);
        }
    }
}

bool mainProcess::getIsFixVelodyneTimeDelay() const
{
    return isFixVelodyneTimeDelay;
}

void mainProcess::setIsFixVelodyneTimeDelay(bool value)
{
    isFixVelodyneTimeDelay = value;
}

QString mainProcess::getNavfilename() const
{
    return navfilename;
}

void mainProcess::setNavfilename(const QString &value)
{
    navfilename = value;
}

/**
 * @brief mainProcess::fixVelodyneTimeDelay 对激光特征进行补偿
 * @param laser
 * @param baseNavIndex
 * @return
 */
bool mainProcess::fixVelodyneTimeDelay(LaserScanData &laser, int& baseNavIndex)
{
    long long laserTime = laser.getTimeStamp().getTickCount();

    ////
    long long baselaserTime = laserTime - 50000; // 50ms
    int pBaseGPS = 0;
    auto pBaseGPSRight = std::lower_bound(navBuf.begin(), navBuf.end(), baselaserTime, [ = ](NavType a, long long b) {
        return a.timestamp < b;
    }) - navBuf.begin();
    if (pBaseGPSRight >= navBuf.size()) {
        progress = 1.0;
        return false;
    }
    auto pBaseGPSLeft = pBaseGPSRight > 0 ? pBaseGPSRight - 1 : pBaseGPSRight;

    if (std::abs(navBuf[pBaseGPSRight].timestamp - baselaserTime) > std::abs(navBuf[pBaseGPSLeft].timestamp - baselaserTime)) {
        pBaseGPS = pBaseGPSLeft;
    } else {
        pBaseGPS = pBaseGPSRight;
    }
    pBaseGPS = pBaseGPSRight;

    auto baseLoc = navBuf[pBaseGPS];
    baseNavIndex = pBaseGPS;

    ///
    int size = laser.points.size();
    for (int i=0; i<laser.points.size(); i++)
    {
        double x = laser.points[i].first;
        double y = laser.points[i].second;

        //第一个激光点在车的正前方,方向逆时针
        long long pointTime = laserTime - 100000.0 / float(size) * (size - i);

        auto pGPSRight = std::lower_bound(navBuf.begin(), navBuf.end(), pointTime, [ = ](NavType a, long long b) {
            return a.timestamp < b;
        }) - navBuf.begin();
        if (pGPSRight >= navBuf.size()) {
            break;
        }
        auto pGPSLeft = pGPSRight > 0 ? pGPSRight - 1 : pGPSRight;
        int pGPS = 0;
        if (std::abs(navBuf[pGPSRight].timestamp - pointTime) > std::abs(navBuf[pGPSLeft].timestamp - pointTime)) {
            pGPS = pGPSLeft;
        } else {
            pGPS = pGPSRight;
        }
        Transform::Point3 p(x, y, 0);
        p = Transform::getPoint(p, "Laser", "GPS");
        Transform::TransformInfo transInfo(-navBuf[pGPS].shv.x, -navBuf[pGPS].shv.y, 0, -navBuf[pGPS].ang.z);
        p.rotate(transInfo);
        p.shift(transInfo);

        //Transform::Point3 p(globalx, globalFrameData.points[i][j].y, globalFrameData.points[i][j].z);
        transInfo = Transform::TransformInfo (baseLoc.shv.x, baseLoc.shv.y, 0, baseLoc.ang.z);
        p.shift(transInfo);
        p.rotate(transInfo);
        p = Transform::getPoint(p, "GPS", "Laser");

        laser.points[i] = std::pair<double,double>(p.x,p.y);
    }

    laser.setTimeStamp(baselaserTime);
    return true;
}

/**
 * @brief mainProcess::fixVelodyneTimeDelay 对原始激光数据进行时间补偿
 * @param inputdata
 * @param outputdata
 * @param lasertime
 * @param basegpsindex
 * @return
 */
bool mainProcess::fixVelodyneTimeDelay(VelodyneRingData &framedata, long long &lasertime, long& basegpsindex)
{
    VelodyneRingData globalframe = framedata;
    for (int i = 0; i < framedata.points.size(); i++) {
        for (int j = 0; j < framedata.points[i].size(); j++) {
            long long pointTime = lasertime - 100000.0 / framedata.points[i].size() * (framedata.points[i].size() - j);
            //long long pointTime = lasertime - 100000.0 / framedata.points[i].size() * (j);
            auto pGPSRight = std::lower_bound(navBuf.begin(), navBuf.end(), pointTime, [ = ](NavType a, long long b) {
                return a.timestamp < b;
            }) - navBuf.begin();
            if (pGPSRight >= navBuf.size()) {
                break;
            }
            auto pGPSLeft = pGPSRight > 0 ? pGPSRight - 1 : pGPSRight;
            int pGPS = 0;
            if (std::abs(navBuf[pGPSRight].timestamp - lasertime) > std::abs(navBuf[pGPSLeft].timestamp - lasertime)) {
                pGPS = pGPSLeft;
            } else {
                pGPS = pGPSRight;
            }
            Transform::Point3 p(framedata.points[i][j].x, framedata.points[i][j].y, framedata.points[i][j].z);
            p = Transform::getPoint(p, "Laser", "GPS");
            Transform::TransformInfo transInfo(-navBuf[pGPS].shv.x, -navBuf[pGPS].shv.y, 0, -navBuf[pGPS].ang.z);
            p.rotate(transInfo);
            p.shift(transInfo);
            globalframe.points[i][j].x = p.x;
            globalframe.points[i][j].y = p.y;
        }
    }

    lasertime = lasertime - 50000; // 50ms
    basegpsindex = 0;
    auto pGPSRight = std::lower_bound(navBuf.begin(), navBuf.end(), lasertime, [ = ](NavType a, long long b) {
        return a.timestamp < b;
    }) - navBuf.begin();
    if (pGPSRight >= navBuf.size()) {
        progress = 1.0;
        return false;
    }
    auto pGPSLeft = pGPSRight > 0 ? pGPSRight - 1 : pGPSRight;

    if (std::abs(navBuf[pGPSRight].timestamp - lasertime) > std::abs(navBuf[pGPSLeft].timestamp - lasertime)) {
        basegpsindex = pGPSLeft;
    } else {
        basegpsindex = pGPSRight;
    }
    basegpsindex = pGPSRight;

    auto midLoc = navBuf[basegpsindex];

    for (int i = 0; i < framedata.points.size(); i++) {
        for (int j = 0; j < framedata.points[i].size(); j++) {
            double x, y, z;
            Transform::Point3 p(globalframe.points[i][j].x, globalframe.points[i][j].y, globalframe.points[i][j].z);
            Transform::TransformInfo transInfo(midLoc.shv.x, midLoc.shv.y, 0, midLoc.ang.z);
            p.shift(transInfo);
            p.rotate(transInfo);
            p = Transform::getPoint(p, "GPS", "Laser");
            x = framedata.points[i][j].x = p.x;
            y = framedata.points[i][j].y = p.y;
            z = framedata.points[i][j].z;
            framedata.distance[i][j] = sqrt(x * x + y * y + z * z);
        }
    }
    return true;
}

bool mainProcess::frameProcess()
{

    bool havedata = this->frameloader.loadOneVeloFrame(framedata);

    if(!havedata){
        progress = 1.0;
        return false;
    }
    VelodyneRingData originFrameData = framedata;

    if (!init) {
        lastLaserTime = framedata.getTimeStamp().getTickCount();
        init = true;
        return true;
    }
    long long laserTime = framedata.getTimeStamp().getTickCount();

    long pGPS = 0;

    if (isFixVelodyneTimeDelay)
    {
        fixVelodyneTimeDelay(framedata, laserTime, pGPS);
    }
    else
    {
        auto pGPSRight = std::lower_bound(navBuf.begin(), navBuf.end(), laserTime, [ = ](NavType a, long long b) {
            return a.timestamp < b;
        }) - navBuf.begin();
        if (pGPSRight >= navBuf.size()) {
            return false;
        }
        auto pGPSLeft = pGPSRight > 0 ? pGPSRight - 1 : pGPSRight;

        if (std::abs(navBuf[pGPSRight].timestamp - laserTime) > std::abs(navBuf[pGPSLeft].timestamp - laserTime)) {
            pGPS = pGPSLeft;
        } else {
            pGPS = pGPSRight;
        }
    }
    progress = float(pGPS)/float(navBuf.size());

    this->savefeature.geneFeatureFiles(framedata);

    // add ds
    drawResult(this->highVis, this->savefeature.labeller.highFeature);
    drawResult(this->midVis, this->savefeature.labeller.midFeature);
    drawResult(this->gndVis, this->savefeature.labeller.gndFeature);
    drawResult(this->laneVis, this->savefeature.labeller.laneFeature);


    //保存路面和车道线特征
    LaserScanData gndfeature = this->savefeature.labeller.gndFeature;
    LaserScanData lanefeature = this->savefeature.labeller.laneFeature;

    std::vector<std::pair<double, double> >gndpts;
    std::vector<std::pair<double, double> >lanepts;
    for (int i=0; i<gndfeature.num; i++)
    {
        if(gndfeature.points[i].first==0&&gndfeature.points[i].second==0)
            continue;
        gndpts.push_back(gndfeature.points[i]);
    }

    for (int i=0; i<lanefeature.num; i++)
    {
        if (lanefeature.points[i].first==0 && lanefeature.points[i].second==0)
            continue;
        lanepts.push_back(lanefeature.points[i]);
    }

    gndDsFile.write((char*)&laserTime, sizeof(long long));
    int size;
    size = gndpts.size();
    gndDsFile.write((char*)&size, sizeof(int));
    for (int i=0; i<gndpts.size(); i++)
    {
        gndDsFile.write((char*)&gndpts[i].first, sizeof(double));
        gndDsFile.write((char*)&gndpts[i].second, sizeof(double));
    }


    laneDsFile.write((char*)&laserTime, sizeof(long long));
    size = lanepts.size();
    laneDsFile.write((char*)&size, sizeof(int));
    for (int i=0; i<lanepts.size(); i++)
    {
        laneDsFile.write((char*)&lanepts[i].first, sizeof(double));
        laneDsFile.write((char*)&lanepts[i].second, sizeof(double));
    }


    std::vector<short> dsvector;
    processLaserScan(savefeature.labeller.highFeature, dsvector);
    highDsFile.write((char*)&navBuf[pGPS].ang, sizeof(point3d));
    highDsFile.write((char*)&navBuf[pGPS].shv, sizeof(point3d));
    highDsFile.write((char*)&navBuf[pGPS].northaccuracy, sizeof(float));
    highDsFile.write((char*)&navBuf[pGPS].eastaccuracy, sizeof(float));
    highDsFile.write((char*)&navBuf[pGPS].downaccuracy, sizeof(float));
    highDsFile.write((char*)&navBuf[pGPS].gpsstatus, sizeof(float));
    //highDsFile.write((char*)&navBuf[pGPS].timestamp, sizeof(long long));
    highDsFile.write((char*)&laserTime, sizeof(long long));
    highDsFile.write((char*)dsvector.data(), sizeof(short)*dsvector.size());

    processLaserScan(savefeature.labeller.midFeature, dsvector);
    midDsFile.write((char*)&navBuf[pGPS].ang, sizeof(point3d));
    midDsFile.write((char*)&navBuf[pGPS].shv, sizeof(point3d));
    midDsFile.write((char*)&navBuf[pGPS].northaccuracy, sizeof(float));
    midDsFile.write((char*)&navBuf[pGPS].eastaccuracy, sizeof(float));
    midDsFile.write((char*)&navBuf[pGPS].downaccuracy, sizeof(float));
    midDsFile.write((char*)&navBuf[pGPS].gpsstatus, sizeof(float));
    //midDsFile.write((char*)&navBuf[pGPS].timestamp, sizeof(long long));
    midDsFile.write((char*)&laserTime, sizeof(long long));
    midDsFile.write((char*)dsvector.data(), sizeof(short)*dsvector.size());


    //    int lineNum = this->savefeature.labeller.highFeature.pointInx[i].first;
    //    int pointNum = savefeature.labeller.highFeature.pointInx[i].second;
    //    if (lineNum==-1 || pointNum==-1)
    //        originFrameData.points[lineNum][pointNum]
    LaserScanData originHighLaser;
    for (int i=0; i<savefeature.labeller.highFeature.pointInx.size(); i++)
    {
        int lineNum = this->savefeature.labeller.highFeature.pointInx[i].first;
        int pointNum = savefeature.labeller.highFeature.pointInx[i].second;
        if (lineNum==-1 || pointNum==-1)
            continue;
        double x = originFrameData.points[lineNum][pointNum].x;
        double y = originFrameData.points[lineNum][pointNum].y;
        originHighLaser.points.push_back(std::pair<double,double>(x,y));
    }
    processLaserScan(originHighLaser, dsvector);
    originHighDsFile.write((char*)&navBuf[pGPS].ang, sizeof(point3d));
    originHighDsFile.write((char*)&navBuf[pGPS].shv, sizeof(point3d));
    originHighDsFile.write((char*)&navBuf[pGPS].northaccuracy, sizeof(float));
    originHighDsFile.write((char*)&navBuf[pGPS].eastaccuracy, sizeof(float));
    originHighDsFile.write((char*)&navBuf[pGPS].downaccuracy, sizeof(float));
    originHighDsFile.write((char*)&navBuf[pGPS].gpsstatus, sizeof(float));
    //midDsFile.write((char*)&navBuf[pGPS].timestamp, sizeof(long long));
    originHighDsFile.write((char*)&laserTime, sizeof(long long));
    originHighDsFile.write((char*)dsvector.data(), sizeof(short)*dsvector.size());

    //qDebug()<<laserTime;
    timestampfile<<navBuf[pGPS].timestamp<<'\t'<<laserTime<<'\t'<<(laserTime - navBuf[pGPS].timestamp)<<std::endl;
    return true;
}

//先提特征后进行激光补偿，但效果没有直接对原始激光补偿好，所以最后放弃该方式
bool mainProcess::frameProcess(int input)
{
    bool havedata = this->frameloader.loadOneVeloFrame(framedata);

    if(!havedata){
        progress = 1.0;
        return false;
    }

    if (!init) {
        lastLaserTime = framedata.getTimeStamp().getTickCount();
        init = true;
        return true;
    }

    long long laserTime = framedata.getTimeStamp().getTickCount();

    this->savefeature.geneFeatureFiles(framedata);

    //保存路面和车道线特征
    LaserScanData gndfeature = this->savefeature.labeller.gndFeature;
    LaserScanData lanefeature = this->savefeature.labeller.laneFeature;
    LaserScanData highfeature = this->savefeature.labeller.highFeature;
    LaserScanData midfeature = this->savefeature.labeller.midFeature;

    int pGPS;
    bool ret = fixVelodyneTimeDelay(gndfeature, pGPS);
    ret &= fixVelodyneTimeDelay(lanefeature, pGPS);
    ret &= fixVelodyneTimeDelay(highfeature, pGPS);
    ret &= fixVelodyneTimeDelay(midfeature, pGPS);
    if (!ret)
        return 0;

    progress = float(pGPS)/float(navBuf.size());

    // add ds
    drawResult(this->highVis, highfeature);
    drawResult(this->midVis, midfeature);
    drawResult(this->gndVis, gndfeature);
    drawResult(this->laneVis, lanefeature);

    std::vector<std::pair<double, double> >gndpts;
    std::vector<std::pair<double, double> >lanepts;
    for (int i=0; i<gndfeature.num; i++)
    {
        if(gndfeature.points[i].first==0&&gndfeature.points[i].second==0)
            continue;
        gndpts.push_back(gndfeature.points[i]);
    }

    for (int i=0; i<lanefeature.num; i++)
    {
        if (lanefeature.points[i].first==0 && lanefeature.points[i].second==0)
            continue;
        lanepts.push_back(lanefeature.points[i]);
    }

    //更新激光时间戳
    laserTime = highfeature.getTimeStamp().getTickCount();

    gndDsFile.write((char*)&laserTime, sizeof(long long));
    int size;
    size = gndpts.size();
    gndDsFile.write((char*)&size, sizeof(int));
    for (int i=0; i<gndpts.size(); i++)
    {
        gndDsFile.write((char*)&gndpts[i].first, sizeof(double));
        gndDsFile.write((char*)&gndpts[i].second, sizeof(double));
    }


    laneDsFile.write((char*)&laserTime, sizeof(long long));
    size = lanepts.size();
    laneDsFile.write((char*)&size, sizeof(int));
    for (int i=0; i<lanepts.size(); i++)
    {
        laneDsFile.write((char*)&lanepts[i].first, sizeof(double));
        laneDsFile.write((char*)&lanepts[i].second, sizeof(double));
    }

    std::vector<short> dsvector;
    processLaserScan(highfeature, dsvector);
    highDsFile.write((char*)&navBuf[pGPS].ang, sizeof(point3d));
    highDsFile.write((char*)&navBuf[pGPS].shv, sizeof(point3d));
    highDsFile.write((char*)&navBuf[pGPS].northaccuracy, sizeof(float));
    highDsFile.write((char*)&navBuf[pGPS].eastaccuracy, sizeof(float));
    highDsFile.write((char*)&navBuf[pGPS].downaccuracy, sizeof(float));
    highDsFile.write((char*)&navBuf[pGPS].gpsstatus, sizeof(float));
    //highDsFile.write((char*)&navBuf[pGPS].timestamp, sizeof(long long));
    highDsFile.write((char*)&laserTime, sizeof(long long));
    highDsFile.write((char*)dsvector.data(), sizeof(short)*dsvector.size());

    processLaserScan(midfeature, dsvector);
    midDsFile.write((char*)&navBuf[pGPS].ang, sizeof(point3d));
    midDsFile.write((char*)&navBuf[pGPS].shv, sizeof(point3d));
    midDsFile.write((char*)&navBuf[pGPS].northaccuracy, sizeof(float));
    midDsFile.write((char*)&navBuf[pGPS].eastaccuracy, sizeof(float));
    midDsFile.write((char*)&navBuf[pGPS].downaccuracy, sizeof(float));
    midDsFile.write((char*)&navBuf[pGPS].gpsstatus, sizeof(float));
    //midDsFile.write((char*)&navBuf[pGPS].timestamp, sizeof(long long));
    midDsFile.write((char*)&laserTime, sizeof(long long));
    midDsFile.write((char*)dsvector.data(), sizeof(short)*dsvector.size());

    //
    processLaserScan(savefeature.labeller.highFeature, dsvector);
    originHighDsFile.write((char*)&navBuf[pGPS].ang, sizeof(point3d));
    originHighDsFile.write((char*)&navBuf[pGPS].shv, sizeof(point3d));
    originHighDsFile.write((char*)&navBuf[pGPS].northaccuracy, sizeof(float));
    originHighDsFile.write((char*)&navBuf[pGPS].eastaccuracy, sizeof(float));
    originHighDsFile.write((char*)&navBuf[pGPS].downaccuracy, sizeof(float));
    originHighDsFile.write((char*)&navBuf[pGPS].gpsstatus, sizeof(float));
    //midDsFile.write((char*)&navBuf[pGPS].timestamp, sizeof(long long));
    originHighDsFile.write((char*)&laserTime, sizeof(long long));
    originHighDsFile.write((char*)dsvector.data(), sizeof(short)*dsvector.size());

    qDebug()<<laserTime;
    timestampfile<<navBuf[pGPS].timestamp<<'\t'<<laserTime<<'\t'<<(laserTime - navBuf[pGPS].timestamp)<<std::endl;
    return true;
}

void mainProcess::drawResult(cv::Mat img, const LaserScanData &data)
{
    img.setTo(0);
    int centerX = img.cols/2;
    int centerY = img.rows/2;
    double pixelSize = 0.5;
    for(int i=0;i!=data.num;i++){
        if(data.points[i].first==0&&data.points[i].second==0)
            continue;
        int imgX = data.points[i].first/pixelSize+centerX;
        int imgY = data.points[i].second/pixelSize+centerY;
        if(imgX<0||imgY<0||imgX>=img.cols||imgX>=img.rows)
            continue;
        cv::circle(img,cv::Point2d(imgX,imgY), 1,cv::Scalar::all(255));
    }
}


