#include "mapdata.h"

MapData::MapData() {
    body.resize(4);
}

MapData::MapData(const MapData &other) {
    for (auto b : other.body) {
        body.push_back(b.clone());
    }
    scene = other.scene;
}

MapBody::MapBody(double initialOx, double initialOy, int width, int height, int pixelSize, bool isNormalized, int type, std::string filename)
    : info(initialOx, initialOy, width, height, pixelSize, filename), isNormalized(isNormalized), type(type) {
    data.create(height, width, type);
    data.setTo(0);

    if (filename != "") {
        load(filename);
    }
}

MapBody::MapBody(MapInfo info, bool isNormalized, int type, std::string filename)
    : info(info), isNormalized(isNormalized), type(type) {
    data.create(info.height, info.width, type);
    data.setTo(0);

    if (filename != "") {
        load(filename);
    }
}

MapBody::MapBody(const MapBody &other) {
    data = other.data.clone();
    info = other.info;
    type = other.type;
    isNormalized = other.isNormalized;
}

bool MapBody::dump(std::string path) {
    if (path != "") {
        info.path = path;
    }
    int freeFlag = type == CV_8U ? -1 : 0;
    if (!isFree(freeFlag)) {
        if (info.filename == "") {
            setFilename();
        }

        FILE *mapFile = fopen(info.filename.c_str(), "wb");
        int size;
        if (data.type() == CV_8U || data.type() == CV_8UC3) {
            size = sizeof(uchar);
        }
        if (data.type() == CV_32F || data.type() == CV_32FC3) {
            size = sizeof(uchar) * 4;
        }
        fwrite(data.data, size, info.width * info.height * data.channels(), mapFile);
        fclose(mapFile);

        // for debug
        cv::Mat singleChannelMat;
        if (data.channels() == 3) {
            std::vector<cv::Mat> channels;
            cv::split(data, channels);
            channels[0].convertTo(singleChannelMat, CV_8U);
        } else if (data.type() == CV_32F) {
            data.convertTo(singleChannelMat, CV_8U, 255);
        } else {
            singleChannelMat = data;
        }
        cv::imwrite(info.filename + ".png", singleChannelMat);
    } else {
        return false;
    }
    return true;
}

bool MapBody::load(std::string name) {
    info.filename = name;
    if (name.substr(name.size() - 4) == ".png") {
        cv::Mat raw;
        raw = cv::imread(name, CV_LOAD_IMAGE_UNCHANGED);
        memcpy(data.data, raw.data, raw.cols * raw.rows * raw.channels());
    } else {
        FILE *mapfile = fopen(info.filename.c_str(), "rb");
        int size;
        if (data.type() == CV_8U || data.type() == CV_8UC3) {
            size = sizeof(uchar);
        }
        if (data.type() == CV_32F || data.type() == CV_32FC3) {
            size = sizeof(uchar) * 4;
        }
        fread(data.data, size, info.height * info.width * data.channels(), mapfile);
        fclose(mapfile);
    }
}

MapBody MapBody::clone() {
    MapBody copy;
    copy.data = data.clone();
    copy.info = info;
    copy.type = type;
    return copy;
}

void MapBody::recreate() {
    data.release();
    data.create(info.height, info.width, type);
    data.setTo(0);
    info.rangeMinX = 0;
    info.rangeMinY = 0;
    info.rangeMaxX = info.width - 1;
    info.rangeMaxY = info.height - 1;
}

std::vector<MapBody> MapBody::split() {
    std::vector<MapBody> result;
    int hCount = info.width / info.unit;
    int vCount = info.height / info.unit;
    for (int i = 0; i < vCount; i++) {
        for (int j = 0; j < hCount; j++) {
            MapBody piece(info.initialOx + info.unit * info.pixelSize * j / 1000,
                          info.initialOy + info.unit * info.pixelSize * i / 1000,
                          info.unit, info.unit, info.pixelSize, true, type);
            cv::Mat srcROI = data(cv::Rect(info.unit * j, info.unit * i, info.unit, info.unit));
            srcROI.copyTo(piece.data);
            piece.info.rangeMinX = info.rangeMinX > info.unit * j ? info.rangeMinX - info.unit * j : 0;
            piece.info.rangeMinY = info.rangeMinY > info.unit * i ? info.rangeMinY - info.unit * i : 0;
            piece.info.rangeMaxX = info.rangeMaxX < info.unit * (j + 1) - 1 ? info.rangeMaxX - info.unit * j : info.unit - 1;
            piece.info.rangeMaxY = info.rangeMaxY < info.unit * (i + 1) - 1 ? info.rangeMaxY - info.unit * i : info.unit - 1;
            result.push_back(piece);
        }
    }
    return result;
}

std::map<std::pair<int, int>, MapInfo> MapBody::getSplitInfo() {
    std::map<std::pair<int, int>, MapInfo> result;
    int hCount = info.width / info.unit;
    int vCount = info.height / info.unit;
    for (int i = 0; i < vCount; i++) {
        for (int j = 0; j < hCount; j++) {
            MapInfo mapInfo(info.initialOx + info.unit * info.pixelSize * j / 1000,
                            info.initialOy + info.unit * info.pixelSize * i / 1000,
                            info.unit, info.unit, info.pixelSize);
            result[std::make_pair(j, i)] = mapInfo;
        }
    }
    return result;
}

void MapBody::setFilename() {
    char buffer[255];
    std::pair<int, int> index = info.getIndex();
    sprintf(buffer, "%d_%d.raw", index.first, index.second);
    info.filename = info.path + std::string(buffer);
}

MapBody MapBody::normalize() {
    MapBody result;
    result.info.pixelSize = info.pixelSize;
    result.isNormalized = true;
    result.type = data.type();

    int newOx = int(ceil((info.ox - info.gridLength + 1) * 1.0 / info.gridLength)) * info.gridLength;
    int newOy = int(ceil((info.oy - info.gridLength + 1) * 1.0 / info.gridLength)) * info.gridLength;

    int deltaX = info.ox - newOx;
    int deltaY = info.oy - newOy;

    int newWidth = int(ceil(deltaX * 1.0 / info.pixelSize + info.width + info.unit - 1) / info.unit) * info.unit;
    int newHeight = int(ceil(deltaY * 1.0 / info.pixelSize + info.height + info.unit - 1) / info.unit) * info.unit;

    result.info.width = newWidth;
    result.info.height = newHeight;
    result.info.ox = newOx;
    result.info.oy = newOy;
    result.info.initialOx = info.initialOx - deltaX / 1000.0;
    result.info.initialOy = info.initialOy - deltaY / 1000.0;
    result.recreate();
    cv::Mat dstROI = result.data(cv::Rect(deltaX / info.pixelSize, deltaY / info.pixelSize, info.width, info.height));
    data.copyTo(dstROI);

    result.info.rangeMinX = deltaX / info.pixelSize;
    result.info.rangeMinY = deltaY / info.pixelSize;
    result.info.rangeMaxX = result.info.rangeMinX + info.width - 1;
    result.info.rangeMaxY = result.info.rangeMinY + info.height - 1;
    return result;
}

bool MapBody::isFree(int channel) {
    if (channel == -1) {
        return cv::countNonZero(data) == 0;
    } else {
        if  (channel >= data.channels()) {
            channel = 0;
        }
        std::vector<cv::Mat> vMat;
        cv::split(data, vMat);
        return cv::countNonZero(vMat[channel]) == 0;
    }
}

MapInfo::MapInfo(double initialOx, double initialOy, int width, int height, int pixelSize, std::string filename, std::string filePath)
    : initialOx(initialOx), initialOy(initialOy), width(width), height(height), pixelSize(pixelSize), filename(filename), path(filePath) {
    if (path != "" && path[path.size() - 1] != '/') {
        path += '/';
    }
    ox = int(round(initialOx * 1000) / pixelSize) * pixelSize; // m -> mm
    oy = int(round(initialOy * 1000) / pixelSize) * pixelSize; // m -> mm
    unit = 1000;
    gridLength = unit * pixelSize;
    rangeMinX = 0;
    rangeMinY = 0;
    rangeMaxX = width - 1;
    rangeMaxY = height - 1;
}

std::pair<int, int> MapInfo::getIndex() {
    int indexX = ox / unit / pixelSize;
    int indexY = oy / unit / pixelSize;
    return std::make_pair(indexX, indexY);
}

MapInfo MapInfo::normalize() {
    MapInfo result;
    result.pixelSize = pixelSize;

    result.ox = int(ceil((ox - gridLength + 1) * 1.0 / gridLength)) * gridLength;
    result.oy = int(ceil((oy - gridLength + 1) * 1.0 / gridLength)) * gridLength;
    int deltaX = ox - result.ox;
    int deltaY = oy - result.oy;

    result.width = int(ceil(deltaX * 1.0 / pixelSize + width + unit - 1) / unit) * unit;
    result.height = int(ceil(deltaY * 1.0 / pixelSize + height + unit - 1) / unit) * unit;

    result.initialOx = initialOx - deltaX / 1000.0;
    result.initialOy = initialOy - deltaY / 1000.0;

    result.rangeMinX = deltaX / pixelSize;
    result.rangeMinY = deltaY / pixelSize;
    result.rangeMaxX = result.rangeMinX + width - 1;
    result.rangeMaxY = result.rangeMinY + height - 1;
    return result;
}
