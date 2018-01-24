#include "timestamp.h"

TimeStamp::TimeStamp() {
    *this = getCurrentTime();
}

TimeStamp::TimeStamp(long long llTimeStamp)
    : llTimeStamp(llTimeStamp) {
    nSec = static_cast<int>(llTimeStamp / 1000000);
    nUsec = llTimeStamp % 1000000;
}

TimeStamp::TimeStamp(int nSec, int nUsec)
    : nSec(nSec), nUsec(nUsec) {
    llTimeStamp = nSec * 1000000 + nUsec;
}

std::string TimeStamp::toString() {
    time_t t = { nSec };
    tm *localTime = localtime(&t);
    char buf[255];
    strftime(buf, 100, "%m-%d %H:%M:%S:", localTime);
    return TypeConverter::toStr(buf) + TypeConverter::toStr(nUsec);
}

bool TimeStamp::operator <(const TimeStamp &other) const {
    return llTimeStamp < other.llTimeStamp;
}

bool TimeStamp::operator >(const TimeStamp &other) const {
    return llTimeStamp > other.llTimeStamp;
}

bool TimeStamp::operator ==(const TimeStamp &other) const {
    return llTimeStamp == other.llTimeStamp;
}

bool TimeStamp::operator !=(const TimeStamp &other) const {
    return llTimeStamp != other.llTimeStamp;
}

long long TimeStamp::getTickCount() const {
    return llTimeStamp;
}

void TimeStamp::setTimeStamp(long long value) {
    llTimeStamp = value;
    nSec = static_cast<int>(llTimeStamp / 1000000);
    nUsec = llTimeStamp % 1000000;
}

void TimeStamp::setTimeStamp(int sec, int usec) {
    nSec = sec;
    nUsec = usec;
    llTimeStamp = nSec * 1000000 + nUsec;
}

int TimeStamp::getNUsec() const {
    return nUsec;
}

int TimeStamp::getNSec() const {
    return nSec;
}

long long TimeStamp::operator -(const TimeStamp &other) {
    return getTickCount() - other.getTickCount();
}

TimeStamp TimeStamp::getCurrentTime() {
    auto now = std::chrono::system_clock::now();
    auto t =  TimeStamp(now.time_since_epoch().count() / 1000);
    return t;
}
