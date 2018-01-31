/** @file
 *  @brief 机器人在线运行框架
 *  @author 鞠孝亮 (JU Xiaoliang)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */


#ifndef LASERPTLABEL_H
#define LASERPTLABEL_H

/**
 * @brief 实现数据标签管理类
 */
class Label {
public:
    enum LabelType {
        Unknown = 0,
        Ground = 1 << 0,
        LaneMarker = 1 << 1,
        Barrier = 1 << 2,
        LeftCurb = 1 << 3,
        RightCurb = 1 << 4,
        LineSeqTerminal = 1 << 5,
        HighObstacle = 1 << 6,
        HighLayerFeature = 1 << 7,
        MidLayerFeature = 1 << 8,
        ObjectOnRoad = 1 << 9,
        Matched = 1 << 10,
        Inlier = 1 << 11
    };/**< 使用二进制位定义可能需要的各个类别 */
    /**
     * @brief 添加B所具备的所有标签
     * @param B 标签对象B
     */
    bool set(Label B);
    /**
     * @brief 添加标签值为B
     * @param B 标签值B
     */
    bool set(LabelType B);
    /**
     * @brief 判断该标签类是否含有标签值B
     * @param B 标签值B
     * @return bool 含有B标签返回true
     */
    bool is(LabelType B) const;
    /**
     * @brief 判断该标签类的所有标签是含有标签类B的所有标签
     * @param B 标签对象B
     * @return bool 包含B返回true
     */
    bool is(Label B) const;
    /**
     * @brief 清空标签类含有的所有标签值
     */
    void reset();
    /**
     * @brief 删除标签类含有的B的所有标签值
     * @param B 标签对象B
     */
    void erase(Label B);
    Label(int type = 0);    
    int type;/**< 标签实际存储位置 */
    Label operator ~();/**< 按位取反运算符 */
    Label operator &(const Label &other);/**< 位与运算符 */
};

#endif // LASERPTLABEL_H
