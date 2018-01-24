#include "savefeature.h"
#include <QDebug>
saveFeature::saveFeature()
{
    this->calibParams.loadCalib("/media/akira/D/SAIC_DATA20170608/db389.xml");
    this->filePath = "/media/akira/D/SAIC_DATA20170725/featureFile";
    highRangeMax = midRangeMax = 50;
    gndRangeMax = 30;
}

saveFeature::~saveFeature()
{
}

void saveFeature::geneFeatureFiles(VelodyneRingData & velodata)
{
    labeller.setRangeMax(highRangeMax,midRangeMax,gndRangeMax);
    labeller.processCurrentFrame(velodata);
}

void saveFeature::setPath(const char *file)
{
    this->filePath = file;
}

void saveFeature::setCalibPath(const char *file)
{
    this->calibParams.loadCalib(file);
    this->labeller.setCalibInfo(this->calibParams);

}

double saveFeature::getHighRangeMax() const
{
    return highRangeMax;
}

void saveFeature::setHighRangeMax(double value)
{
    highRangeMax = value;
}

double saveFeature::getMidRangeMax() const
{
    return midRangeMax;
}

void saveFeature::setMidRangeMax(double value)
{
    midRangeMax = value;
}

double saveFeature::getGndRangeMax() const
{
    return gndRangeMax;
}

void saveFeature::setGndRangeMax(double value)
{
    gndRangeMax = value;
}
