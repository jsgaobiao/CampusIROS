#include "label.h"

Label::Label(int type)
    : type(type) {}

bool Label::set(Label B) {
    /*
     * 使用位或运算符实现标签设置
     */
    type = B.type | type;
    return true;
}

bool Label::set(LabelType B) {
    /*
     * 使用位或运算符实现标签设置
     */
    type = B | type;
    return true;
}

bool Label::is(LabelType A) const {
    /*
     * 使用标签存储所在的int整数判断两包含关系
     */
    if(A == LabelType::Unknown) {
        return type == 0;
    } else {
        return static_cast<int>(type & A) == static_cast<int>(A);
    }
}
bool Label::is(Label A) const {
    /*
     * 使用标签存储所在的int整数判断两包含关系
     */
    if(A.type == LabelType::Unknown) {
        return type == 0;
    } else {
        return static_cast<int>(type & A.type) == static_cast<int>(A.type);
    }
}

void Label::reset() {
    /*
     * 重置为UNKOWN标签
     */
    type = 0;
}

void Label::erase(Label B) {
    /*
     * 使用位与运算符实现标签设置
     */
    type &= ~B.type;
}

Label Label::operator &(const Label &other) {
    /*
     * 实现位与运算符
     */
    type &= other.type;
    return *this;
}

Label Label::operator ~() {
    /*
     * 实现按位取反运算符
     */
    Label ans(~type);
    return ans;
}
