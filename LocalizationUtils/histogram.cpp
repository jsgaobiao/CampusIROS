#include "histogram.h"

HistoGram::HistoGram()
{
    /*
     * 初始化各统计值为0.0
    */
    this->totalDataNum = 0;
    this->meanValue = 0.0;
    this->varValue = 0.0;
    this->meanSquareValue = 0.0;
}

void HistoGram::clear()
{
    /*
     * 清空统计数据并初始化为0.0
    */
    this->rangeList.clear();
    this->totalDataNum = 0;
    this->meanValue = 0.0;
    this->varValue = 0.0;
    this->meanSquareValue = 0.0;
}

void HistoGram::addValue(double value)
{
    /*
     * 向该直方图统计添加新数据，更新统计值
    */
    this->meanSquareValue = (double)totalDataNum / (totalDataNum + 1) * meanSquareValue + value * value / (totalDataNum + 1);
    this->meanValue = (double)totalDataNum / (totalDataNum + 1) * meanValue + value / (totalDataNum + 1);
    this->varValue = meanSquareValue - meanValue * meanValue;
    this->totalDataNum++;
    for(auto &range : this->rangeList){
        if(value >= std::get<0>(range) && value < std::get<1>(range)){
            std::get<2>(range)++;
        }
    }
}

void HistoGram::addRange(double lower, double upper)
{
    /*
     * 向该直方图统计添加新数据统计范围
    */
    this->rangeList.push_back(std::make_tuple(lower, upper, 0, 0.0));
}

void HistoGram::calPercentage()
{
    /*
     * 必要时计算数据统计的百分数
    */
    for(auto &range : this->rangeList){
        std::get<3>(range) = (double) std::get<2>(range) / totalDataNum;
    }

}

std::string HistoGram::geneHistResult()
{
    /*
     * 生成统计直方图的文字结果
    */
    this->calPercentage();
    std::stringstream strStream;
    strStream << "\nRangeLowerBound" << '\t' << "RangeUpperBound" << '\t' << "Frequency" << '\t' << "Percentage\n";
    for(auto range : this->rangeList){
        strStream << std::get<0>(range) << '\t' << std::get<1>(range) << '\t' << std::get<2>(range) << '\t' << std::get<3>(range) << '\n';
    }
    return strStream.str();
}
