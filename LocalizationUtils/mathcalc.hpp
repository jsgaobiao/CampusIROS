/** @file
 *  @brief 机器人在线运行框架
 *  @author 鞠孝亮 (JU Xiaoliang)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */

#ifndef MATHCALC_H
#define MATHCALC_H

#include <cmath>
#include <vector>
/**
 * @brief 提供部分常用基本运算
 *
 */
namespace MathCalc {

template<class A>
/**
 * @brief 限定x范围给出x值
 *
 * @param x 输入
 * @param min 下界
 * @param max 上界
 * @return A 限定后结果
 */
A bound(A x, A min, A max) {
    return (x) < (min) ? (min) : ((x) > (max) ? (max) : (x));
}

template<class A>
/**
 * @brief 求取平方数
 *
 * @param x 输入
 * @return A 平方结果
 */
A sqr(A x) {
    return x * x;
}

template<class A>
/**
 * @brief 求取进位整值
 *
 * @param x 输入
 * @return int 取整值
 */
int nint(A x) {
    return static_cast<int>((x > 0) ? (x + 0.5) : (x - 0.5));
}

template<class A>
/**
 * @brief 求取限定范围均值
 *
 * @param insig 输入序列
 * @param start 计算起始点
 * @param end 计算终止点
 * @return A 均值
 */
A meanforVector(std::vector<A> const &insig, std::size_t start, std::size_t end) {
    A sum = 0;
    int cnt = 0;
    for (auto &c : insig) {
        if (cnt < start) {
            cnt++;
            continue;
        }
        if (cnt > end) {
            break;
        }
        sum += c;
        cnt++;
    }
    return sum / (end - start);
}

template<class A>
/**
 * @brief 求取限定范围标准差
 *
 * @param insig 输入序列
 * @param start 计算起始点
 * @param end 计算终止点
 * @return A 标准差
 */
A dispersionforVec(std::vector<A>const &insig, std::size_t start, std::size_t end) {
    A aver = meanforVector(insig, start, end);
    A sum = 0;
    int cnt = 0;
    for (auto &c : insig) {
        if (cnt < start) {
            cnt++;
            continue;
        }
        if (cnt > end) {
            break;
        }
        sum += (c - aver) * (c - aver);
        cnt++;
    }
    return sqrt(sum / (end - start));
}

template<class A>
/**
 * @brief 滑动均值滤波
 *
 * @param seqin 输入信号
 * @param seqout 输出信号
 * @param wid 滑动窗宽度
 */
void smoothseq(std::vector<A>&seqin, std::vector<A>&seqout, std::size_t wid) {
    std::vector<A>tmp;
    for (std::size_t i = 0; i != seqin.size(); i++) {
        A sum = 0; int cnt = 0;
        for (int j = i - wid / 2; j != i + wid / 2 + 1; j++) {
            if (j >= 0 && j < seqin.size()) {
                sum += seqin[j];
                cnt++;
            }
        }
        tmp.push_back(round(sum / cnt));
    }
    seqout.clear();
    seqout = tmp;
}

template<class A>// limit class A has precision <float/double> members
/**
 * @brief 计算三点构成平面参数
 *
 * @param ptsForFit 三点序列
 * @param a 平面参数满足 a * x + b * y + c * z + d = 0.0
 * @param b
 * @param c
 * @param d
 * @return bool 输入正确标志,true为输入正确,false为输入有误
 */
bool calPlane(std::vector<A> &ptsForFit, double &a, double &b, double &c, double &d) {
    if (ptsForFit.size() != 3) {
        return false;
    }
    A p1, p2, p3;

    p1 = ptsForFit[0];
    p2 = ptsForFit[1];
    p3 = ptsForFit[2];

    a = ((p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y));
    b = ((p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z));
    c = ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x));
    d = (0 - (a * p1.x + b * p1.y + c * p1.z));

    return true;
}

template<class A>// limit class A has precision <float/double> members
/**
 * @brief 计算点到平面欧氏距离
 *
 * @param p1 输入点
 * @param a 平面参数满足 a * x + b * y + c * z + d = 0.0
 * @param b
 * @param c
 * @param d
 * @return double 计算距离结果
 */
double DistPt2Panel(A p1, double a, double b, double c, double d) {
    return fabs(a * p1.x + b * p1.y + c * p1.z + d) / sqrt(a * a + b * b + c * c);
}

// limit class A has precision <float/double> members
template<class A>
/**
 * @brief 计算点到平面欧氏距离(带符号)
 *
 * @param p1 输入点
 * @param a 平面参数满足 a * x + b * y + c * z + d = 0.0
 * @param b
 * @param c
 * @param d
 * @return double 计算距离结果
 */
double SignedDistPt2Panel(A p1, double a, double b, double c, double d) {
    double dis = (a * p1.x + b * p1.y + c * p1.z + d) / sqrt(a * a + b * b + c * c);
    if (c < 0) {
        dis = -dis;
    }
    return dis;
}

template<class A>
/**
 * @brief 计算二维点到直线距离(带符号)
 *
 * @param a 输入点
 * @param para 直线参数数组指针,满足para[0] * x + para[1] * y + para[2] =0
 * @return double 距离值
 */
double calSignedDisPt2Line2D(A a, double *para) {
    return (para[0] * a.x + para[1] * a.y + para[2]) / sqrt(sqr(para[0]) + sqr(para[1]));
}

template<class A>
/**
 * @brief calMinAngleDifferce 计算a，b两个角度差值
 * @param a 被减数
 * @param b 减数
 * @return 角度差值，其绝对值小于M_PI
 */
A calMinAngleDifferce(A a, A b) {
    a = atan2(sin(a), cos(a));
    b = atan2(sin(b), cos(b));
    A diff = a - b;
    if(fabs(diff) > M_PI){
        if(diff > 0.0){
            return (diff - 2 * M_PI);
        }
        else{
            return (2 * M_PI + diff);
        }
    }
    return diff;
}


}

#endif // MATHCALC_H
