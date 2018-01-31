#include "TrajCapture.h"

bool comp(TrajData &a, TrajData &b)
{
    return a.timestamp < b.timestamp;
}

TrajCapture::TrajCapture(std::string fin)
{
    data.clear();

    std::fstream in(fin, std::ios::in);
    char buffer[256];
    int _tno = 0;
    TrajData newData;

    if (in.is_open()) {
        // Table Header
        in.getline(buffer, 200);
        while (! in.eof()) {
            in.getline(buffer, 200);
            if (std::strlen(buffer) > 0 && std::strlen(buffer) < 30) {
                std::sscanf(buffer, "tno=%d\n", &_tno);
            }
            else {
                newData.tno = _tno;
                std::sscanf(buffer, "%d,%*d,%*lf,%*lf,%*lf,%*lf,%*lf,%*lf,%lf,%lf,%*lf,%*lf\n", &newData.timestamp, &newData.lx, &newData.ly);
                data.push_back(newData);
            }
        }
        std::sort(data.begin(), data.end(), comp);
    }
    else {
        printf("TrajCapture Open Error\n");
    }
}
