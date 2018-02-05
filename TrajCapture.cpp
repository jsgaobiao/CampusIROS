#include "TrajCapture.h"

bool comp(TrajData &a, TrajData &b)
{
    return a.timestamp < b.timestamp;
}

TrajCapture::TrajCapture(std::string fin, std::string fWhiteList)
{
    data.clear();

    std::fstream in(fin, std::ios::in);
    std::fstream whiteListIn(fWhiteList, std::ios::in);
    bool flagList[10000];
    std::memset(flagList, 0, sizeof(flagList));
    if (whiteListIn.is_open()) {
        while (! whiteListIn.eof()) {
            int number;
            whiteListIn >> number;
            if (number >= 0 && number < 10000)
                flagList[number] = true;
        }
    }
    whiteListIn.close();

    char buffer[256];
    int _tno = 0;
    TrajData newData;
    int cntFrame = 0;

    if (in.is_open()) {
        // Table Header
        in.getline(buffer, 200);
        while (! in.eof()) {
            in.getline(buffer, 200);
            if (std::strlen(buffer) > 0 && std::strlen(buffer) < 30) {
//                if (cntFrame > 0 && cntFrame < 15) {
//                    for (int i = 0; i < cntFrame; i ++) {
//                        data.pop_back();
//                    }
//                }
                std::sscanf(buffer, "tno=%d\n", &_tno);
                cntFrame = 0;
            }
            else {
                newData.tno = _tno;
                cntFrame ++;
                std::sscanf(buffer, "%d,%*d,%*lf,%*lf,%*lf,%*lf,%*lf,%*lf,%lf,%lf,%*lf,%*lf\n", &newData.timestamp, &newData.lx, &newData.ly);
                if (flagList[_tno]) {
                    newData.isPedestrian = true;
                }
                else {
                    newData.isPedestrian = false;
                }
                data.push_back(newData);
            }
        }
        std::sort(data.begin(), data.end(), comp);
    }
    else {
        printf("TrajCapture Open Error\n");
    }

    in.close();
}
