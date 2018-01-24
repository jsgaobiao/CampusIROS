#include "localizationstatistics.h"

LocalizationStatistics::LocalizationStatistics()
{
    /*
     * 构造时即设定统计范围，一般无需更改
     */
    setDefaultRangeLists();
}

void LocalizationStatistics::setDefaultRangeLists()
{
    HistoGram *hist[7] = {&error2GPSHist, &error2GPS_lateralHist, &error2GPS_longtitudeHist, &absError2GPS_lateralHist, &absError2GPS_longtitudeHist,
                         &error2GPS_yawHist, &absError2GPS_yawHist};
    for(int i = 0; i != 5; i++){
        hist[i]->addRange(0.0, 0.2);
        hist[i]->addRange(0.2, 0.4);
        hist[i]->addRange(0.4, 0.6);
        hist[i]->addRange(0.6, 0.8);
        hist[i]->addRange(0.8, 1.0);
        hist[i]->addRange(1.0, 100000.0);
    }
    for(int i = 1; i != 3; i++){
        hist[i]->addRange(-0.2, 0.0);
        hist[i]->addRange(-0.4, -0.2);
        hist[i]->addRange(-0.6, -0.4);
        hist[i]->addRange(-0.8, -0.6);
        hist[i]->addRange(-1.0, -0.8);
        hist[i]->addRange(-100000, -1.0);
    }

    for(int i = 5; i != 7; i++){
        hist[i]->addRange(0.0, 0.25);
        hist[i]->addRange(0.25, 0.5);
        hist[i]->addRange(0.5, 1);
        hist[i]->addRange(1, 2);
        hist[i]->addRange(2, 10);
        hist[i]->addRange(10, 100000.0);
    }
    for(int i = 5; i != 6; i++){
        hist[i]->addRange(-0.25, 0.0);
        hist[i]->addRange(-0.5, -0.25);
        hist[i]->addRange(-1, -0.5);
        hist[i]->addRange(-2, -1);
        hist[i]->addRange(-10, -2);
        hist[i]->addRange(-100000.0, -10);
    }


}

void LocalizationStatistics::update(const LocationResult &locationResult)
{
    this->error2GPSHist.addValue(locationResult.error2GPS);
    this->error2GPS_lateralHist.addValue(locationResult.error2GPS_lateral);
    this->error2GPS_longtitudeHist.addValue(locationResult.error2GPS_longti);
    this->absError2GPS_lateralHist.addValue(fabs(locationResult.error2GPS_lateral));
    this->absError2GPS_longtitudeHist.addValue(fabs(locationResult.error2GPS_longti));
    /*
     * 统计角度误差值
     */
    this->error2GPS_yawHist.addValue(locationResult.error2GPS_yaw * 180 / M_PI);
    this->absError2GPS_yawHist.addValue(fabs(locationResult.error2GPS_yaw * 180 / M_PI));

}
