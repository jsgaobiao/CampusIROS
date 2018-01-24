#include "transform.h"

namespace Transform {

bool existFrame(std::string frame) {
    for (auto name: frameList) {
        if (name == frame) {
            return true;
        }
    }
    return false;
}

bool existLink(std::string src, std::string dst) {
    return infoList.find(std::make_pair(src, dst)) != infoList.cend()
           && infoList[std::make_pair(src, dst)].valid;
}

void clear() {
    frameList.clear();
    infoList.clear();
}

bool update(std::string src, std::string dst) {
    std::vector<std::string> srcFrameList, dstFrameList;
    for (auto frame: frameList) {
        if (frame != src && frame != dst && existLink(src, frame)) {
            srcFrameList.push_back(frame);
        }
        if (frame != src && frame != dst && existLink(dst, frame)) {
            dstFrameList.push_back(frame);
        }
    }
    for (auto frame: dstFrameList) {
        infoList[std::make_pair(src, frame)] = infoList[std::make_pair(src, dst)] + infoList[std::make_pair(dst, frame)];
        infoList[std::make_pair(frame, src)] = -infoList[std::make_pair(src, frame)];
    }
    dstFrameList.push_back(dst);
    for (auto frame1: srcFrameList) {
        for (auto frame2: dstFrameList) {
            infoList[std::make_pair(frame1, frame2)] = infoList[std::make_pair(frame1, src)] + infoList[std::make_pair(src, frame2)];
            infoList[std::make_pair(frame2, frame1)] = -infoList[std::make_pair(frame1, frame2)];
        }
    }
    return true;
}

bool addInfo(TransformInfo info, std::string src, std::string dst) {
    if (existLink(src, dst)) {
        return infoList[std::make_pair(src, dst)] == info;
    }
    infoList[std::make_pair(src, src)] = TransformInfo();
    infoList[std::make_pair(dst, dst)] = TransformInfo();
    infoList[std::make_pair(src, dst)] = info;
    infoList[std::make_pair(dst, src)] = -info;
    if (!existFrame(src)) {
        frameList.push_back(src);
    }
    if (!existFrame(dst)) {
        frameList.push_back(dst);
    }
    update(src, dst);
    return true;
}

Point3 getPoint(Point3 p, std::string src, std::string dst) {
    if (!existLink(src, dst)) {
        return Point3(false);
    }
    Point3 ans = p;
    TransformInfo info = infoList[std::make_pair(src, dst)];
    ans = transformPoint(p, info.rot, info.shift);
    return ans;
}

TransformInfo getInfo(std::string src, std::string dst) {
    if (!existLink(src, dst)) {
        return TransformInfo(false);
    }
    return infoList[std::make_pair(src, dst)];
}

Point3 shiftPoint(Point3 p, double x, double y, double z) {
    Point3 ans = p;
    ans.x -= x;
    ans.y -= y;
    ans.z -= z;
    return ans;
}

Point3 shiftPoint(Point3 p, Eigen::Vector3d trans) {
    Point3 ans = p;
    ans.x += trans[0];
    ans.y += trans[1];
    ans.z += trans[2];
    return ans;
}

Point3 rotatePoint(Point3 p, double yaw, double pitch, double roll) {
    Point3 ans = p;
    Eigen::Matrix3d mat;
    double sin_yaw = sin(-yaw), cos_yaw = cos(-yaw);
    double sin_pitch = sin(-pitch), cos_pitch = cos(-pitch);
    double sin_roll = sin(-roll), cos_roll = cos(-roll);
    mat(0, 0) = cos_yaw * cos_pitch;
    mat(0, 1) = -sin_yaw * cos_roll - cos_yaw * sin_pitch * sin_roll;
    mat(0, 2) = sin_yaw * sin_roll - cos_yaw * sin_pitch * cos_roll;
    mat(1, 0) = sin_yaw * cos_pitch;
    mat(1, 1) = cos_yaw * cos_roll - sin_yaw * sin_pitch * sin_roll;
    mat(1, 2) = -cos_yaw * sin_roll - sin_yaw * sin_pitch * cos_roll;
    mat(2, 0) = sin_pitch;
    mat(2, 1) = cos_pitch * sin_roll;
    mat(2, 2) = cos_pitch * cos_roll;
    ans.x = mat(0, 0) * p.x + mat(0, 1) * p.y + mat(0, 2) * p.z;
    ans.y = mat(1, 0) * p.x + mat(1, 1) * p.y + mat(1, 2) * p.z;
    ans.z = mat(2, 0) * p.x + mat(2, 1) * p.y + mat(2, 2) * p.z;
    return ans;
}

Point3 rotatePoint(Point3 p, Eigen::Matrix3d mat) {
    Point3 ans = p;
    ans.x = mat(0, 0) * p.x + mat(0, 1) * p.y + mat(0, 2) * p.z;
    ans.y = mat(1, 0) * p.x + mat(1, 1) * p.y + mat(1, 2) * p.z;
    ans.z = mat(2, 0) * p.x + mat(2, 1) * p.y + mat(2, 2) * p.z;
    return ans;
}

Point3 transformPoint(Point3 p, Eigen::Matrix3d rot, Eigen::Vector3d shift) {
    Point3 p1 = p, ans;
    p1.x += shift(0);
    p1.y += shift(1);
    p1.z += shift(2);
    ans.x = rot(0, 0) * p1.x + rot(0, 1) * p1.y + rot(0, 2) * p1.z;
    ans.y = rot(1, 0) * p1.x + rot(1, 1) * p1.y + rot(1, 2) * p1.z;
    ans.z = rot(2, 0) * p1.x + rot(2, 1) * p1.y + rot(2, 2) * p1.z;
    return ans;
}

TransformInfo::TransformInfo(bool valid)
    : valid(valid) {
    rot(0, 0) = 1;
    rot(0, 1) = 0;
    rot(0, 2) = 0;
    rot(1, 0) = 0;
    rot(1, 1) = 1;
    rot(1, 2) = 0;
    rot(2, 0) = 0;
    rot(2, 1) = 0;
    rot(2, 2) = 1;
    shift(0) = 0;
    shift(1) = 0;
    shift(2) = 0;
}

TransformInfo::TransformInfo(double x, double y, double z, double yaw, double pitch, double roll, bool valid)
    : valid(valid) {
    double sin_yaw = sin(-yaw), cos_yaw = cos(-yaw);
    double sin_pitch = sin(-pitch), cos_pitch = cos(-pitch);
    double sin_roll = sin(-roll), cos_roll = cos(-roll);
    rot(0, 0) = cos_yaw * cos_pitch;
    rot(0, 1) = -sin_yaw * cos_roll - cos_yaw * sin_pitch * sin_roll;
    rot(0, 2) = sin_yaw * sin_roll - cos_yaw * sin_pitch * cos_roll;
    rot(1, 0) = sin_yaw * cos_pitch;
    rot(1, 1) = cos_yaw * cos_roll - sin_yaw * sin_pitch * sin_roll;
    rot(1, 2) = -cos_yaw * sin_roll - sin_yaw * sin_pitch * cos_roll;
    rot(2, 0) = sin_pitch;
    rot(2, 1) = cos_pitch * sin_roll;
    rot(2, 2) = cos_pitch * cos_roll;
    shift(0) = -x;
    shift(1) = -y;
    shift(2) = -z;
}

TransformInfo::TransformInfo(Eigen::Matrix3d rot, Eigen::Vector3d shift, bool valid)
    : rot(rot), shift(shift), valid(valid) {}

bool TransformInfo::operator==(const TransformInfo &other) {
    return (rot - other.rot).norm() <= eps && (shift - other.shift).norm() <= eps;
}

TransformInfo TransformInfo::operator-() {// minus
    TransformInfo info;
    info.shift = -rot * shift;
    info.rot = rot.inverse();
    return info;
}

TransformInfo TransformInfo::operator+(const TransformInfo &other) {
    TransformInfo info;
    info.rot = other.rot * rot;
    info.shift = rot.inverse() * other.shift + shift;
    return info;
}

void Point3::rotate(TransformInfo info) {
    *this = rotatePoint(*this, info.rot);
}

void Point3::shift(TransformInfo info) {
    *this = shiftPoint(*this, info.shift);
}

}
