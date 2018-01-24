#include "basedata.h"

BaseData::BaseData() {
    timeStamp = TimeStamp::getCurrentTime();
}

BaseData::~BaseData() {}

void BaseData::setTimeStamp(const TimeStamp &value) {
    timeStamp = value;
}

TimeStamp BaseData::setTimeStamp() {
    timeStamp = TimeStamp::getCurrentTime();
    return timeStamp;
}

TimeStamp BaseData::getTimeStamp() const {
    return timeStamp;
}

string BaseData::toString() {
    return TypeConverter::toStr(typeid(*this).name()) + " generated at: " + timeStamp.toString();
}
