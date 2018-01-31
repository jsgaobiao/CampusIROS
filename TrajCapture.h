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
};

class TrajCapture
{
public:
    TrajCapture(std::string fin);

public:
    std::vector<TrajData> data;
};

#endif // TRAJCAPTURE_H
