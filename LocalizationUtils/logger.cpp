#include "logger.h"

void Logger::startCalTime(std::string name) {
    if (!enableTimePrint) {
        return;
    }
    timeLogStarttime[name] = TimeStamp::getCurrentTime();
}

double Logger::stopCalTime(std::string name) {
    if (!enableTimePrint) {
        return -1;
    }
    if (timeLogStarttime.find(name) == timeLogStarttime.end()) {
        return -1;
    }
    auto start = timeLogStarttime[name];
    auto now = TimeStamp::getCurrentTime();
    double timeUsed = (now - start) / 1e3;
    qDebug() << QString("[%1]: %2 ms").arg(name.c_str()).arg(timeUsed);
    return timeUsed;
}

void Logger::setLogFile(std::string name, std::string path) {
    if (logPath.find(path) != logPath.end()) {
        logStream[name] = logPath[path];
    } else {
        std::ofstream *ptr = new std::ofstream(path, std::ios::app);
        auto now = TimeStamp::getCurrentTime();
        *ptr << QString("Opened at [%1]\n").arg(now.toString().c_str()).toStdString();
        auto fPtr = std::shared_ptr<std::ofstream>(ptr, [ = ](std::ofstream* ptr) {
            auto now = TimeStamp::getCurrentTime();
            *ptr << QString("Closed at [%1]\n").arg(now.toString().c_str()).toStdString();
            ptr->close();
            delete ptr;
        });
        logStream[name] = fPtr;
        logPath[path] = fPtr;
    }
}

void Logger::setLogDumpStatus(bool enable) {
    enableLogDump = enable;
}

void Logger::setTimePrintStatus(bool enable) {
    enableTimePrint = enable;
}

void Logger::setLogPrintStatus(bool enable) {
    enableLogPrint = enable;
}

void Logger::printLog(std::string name, long long message) {
    printLog(name, QString::number(message).toStdString());
}

void Logger::clearLogFile() {
    logPath.clear();
    logStream.clear();
}

void Logger::printLog(std::string name, std::string message, bool outputTime) {
    if (!enableLogPrint && !enableLogDump) {
        return;
    }
    if (logStream.find(name) == logStream.end()) {
        qDebug() << name.c_str() << "Logger doesn't exist.";
        return;
    }
    auto now = TimeStamp::getCurrentTime();
    if (enableLogDump) {
        auto ptr = logStream[name];
        if (outputTime) {
            *ptr << QString("[%1]: %2\n").arg(now.getTickCount()).arg(message.c_str()).toStdString();
        } else {
            *ptr << message + '\n';
        }
        ptr->flush();
    }
    if (enableLogPrint) {
        qDebug() << QString("[%1]: %2").arg(name.c_str()).arg(message.c_str());
    }
}
