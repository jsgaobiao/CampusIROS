#ifndef TRAJCAPTURE_H
#define TRAJCAPTURE_H
#include <cstdio>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <cstring>

class TrajData
{
public:
    int tno;
    int timestamp;
    double lx;
    double ly;
    bool isPedestrian;
};

class TrajCapture
{
public:
    TrajCapture(std::string fin, std::string fwhiteList);

public:
    std::vector<TrajData> data;
};

#endif // TRAJCAPTURE_H
