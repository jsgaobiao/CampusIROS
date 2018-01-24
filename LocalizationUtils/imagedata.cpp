#include "imagedata.h"

ImageData::ImageData() {}

ImageData::ImageData(const ImageData &data) {
    image = data.image.clone();
    setTimeStamp(data.getTimeStamp());
}
