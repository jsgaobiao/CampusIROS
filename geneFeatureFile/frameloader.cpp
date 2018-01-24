#include "frameloader.h"
#include <QDebug>
FrameLoader::FrameLoader()
{
    this->logFile = nullptr;
    this->lineNum = 64;
    this->maxLinePointNum = 2200;
}

void FrameLoader::setPath(const char *path)
{
    this->logFile = fopen(path,"rb");
    input_xyzi_filename = QString(path);

    qDebug()<<path;
}


bool FrameLoader::loadOneVeloFrame(VelodyneRingData &oneframe)
{
    oneframe.clear();
    long long timeStamp;
    int readCnt = fread((char*)(&timeStamp), sizeof(timeStamp), 1, logFile);
    if(readCnt!=1)
        return false;
    char buffer[2097152];
    readCnt = fread((char*)(buffer), sizeof(char), (4 + 13 * maxLinePointNum) * lineNum, logFile);
    if(readCnt!=(4 + 13 * maxLinePointNum) * lineNum)
        return false;
    for (size_t i = 0; i < lineNum; i++) {
        int linePointNum = *(int*)&buffer[i * (4 + 13 * maxLinePointNum)];
        for (size_t j = 0; j < maxLinePointNum; j++) {
            float x, y, z;
            char intensity;
            x = *(float*)&buffer[4 + i * (4 + 13 * maxLinePointNum) + j * 13];
            y = *(float*)&buffer[8 + i * (4 + 13 * maxLinePointNum) + j * 13];
            z = *(float*)&buffer[12 + i * (4 + 13 * maxLinePointNum) + j * 13];
            intensity = *(char*)&buffer[16 + i * (4 + 13 * maxLinePointNum) + j * 13];
            if (j < linePointNum) {
                oneframe.points[i].push_back(cv::Point3d(x, y, z));
                oneframe.distance[i].push_back(sqrt(x * x + y * y + z * z));
                oneframe.intensity[i].push_back(intensity);
                oneframe.label[i].push_back(Label(Label::Unknown));
                oneframe.enable[i].push_back(1);
            }
        }
    }
    oneframe.setTimeStamp(TimeStamp(timeStamp));
    return true;
}
