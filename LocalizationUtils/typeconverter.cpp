#include "typeconverter.h"

std::string TypeConverter::toStr(const char *src) {
    return std::string(src);
}

std::string TypeConverter::toStr(long long src) {
    std::stringstream stream;
    stream << src;
    return stream.str();
}

std::vector<std::string> TypeConverter::splitStr(std::string str, char sep) {
    std::vector<std::string> strList;
    size_t last = 0;
    size_t index = str.find_first_of(sep, last);
    while (index != std::string::npos) {
        if (index - last > 0) {
            strList.push_back(str.substr(last, index - last));
        }
        last = index + 1;
        index = str.find_first_of(sep, last);
    }
    if (last != str.size() && index - last > 0) {
        strList.push_back(str.substr(last, index - last));
    }
    return strList;
}
