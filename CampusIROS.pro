QT     += core gui xml widgets
TEMPLATE = app
CONFIG += console c++11

#BOOST
LIBS += -lboost_system

#PCL
LIBS += -lpcl_common -lpcl_kdtree -lpcl_search -lpcl_features -lpcl_segmentation
INCLUDEPATH += /usr/local/include/pcl-1.8/

#PCAP
LIBS += -lpcap

#OPENCV3
INCLUDEPATH += /usr/local/include/
LIBS += -L/usr/local/lib -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lopencv_videoio -lopencv_viz

INCLUDEPATH += LocalizationUtils
DEPENDPATH += LocalizationUtils

INCLUDEPATH += FeatureExtractor
DEPENDPATH += FeatureExtractor

#EIGEN
INCLUDEPATH += /usr/include/eigen3

SOURCES += main.cpp \
    FeatureExtractor/grounddetector.cpp \
    FeatureExtractor/lanemarkerdetector.cpp \
    FeatureExtractor/laserlineseq.cpp \
    FeatureExtractor/locallaserstatsmap.cpp \
    FeatureExtractor/planefitting.cpp \
    FeatureExtractor/velodatalabeller.cpp \
    geneFeatureFile/frameloader.cpp \
    geneFeatureFile/mainprocess.cpp \
    geneFeatureFile/mainwindow.cpp \
    geneFeatureFile/savefeature.cpp \
    LocalizationUtils/basedata.cpp \
    LocalizationUtils/histogram.cpp \
    LocalizationUtils/imagedata.cpp \
    LocalizationUtils/label.cpp \
    LocalizationUtils/laserscandata.cpp \
    LocalizationUtils/localizationstatistics.cpp \
    LocalizationUtils/locationresult.cpp \
    LocalizationUtils/logger.cpp \
    LocalizationUtils/mapdata.cpp \
    LocalizationUtils/navdata.cpp \
    LocalizationUtils/particlesetdata.cpp \
    LocalizationUtils/timestamp.cpp \
    LocalizationUtils/transform.cpp \
    LocalizationUtils/typeconverter.cpp \
    LocalizationUtils/vcudata.cpp \
    LocalizationUtils/velocalib.cpp \
    LocalizationUtils/velodyneringdata.cpp \
    ECSegmentation.cpp \
    TrajCapture.cpp

FORMS += \
    geneFeatureFile/mainwindow.ui

HEADERS += \
    FeatureExtractor/grounddetector.h \
    FeatureExtractor/lanemarkerdetector.h \
    FeatureExtractor/laserlineseq.h \
    FeatureExtractor/locallaserstatsmap.h \
    FeatureExtractor/planefitting.h \
    FeatureExtractor/velodatalabeller.h \
    geneFeatureFile/frameloader.h \
    geneFeatureFile/mainprocess.h \
    geneFeatureFile/mainwindow.h \
    geneFeatureFile/savefeature.h \
    LocalizationUtils/basedata.h \
    LocalizationUtils/histogram.h \
    LocalizationUtils/imagedata.h \
    LocalizationUtils/label.h \
    LocalizationUtils/laserscandata.h \
    LocalizationUtils/localizationstatistics.h \
    LocalizationUtils/locationresult.h \
    LocalizationUtils/logger.h \
    LocalizationUtils/mapdata.h \
    LocalizationUtils/mathcalc.hpp \
    LocalizationUtils/navdata.h \
    LocalizationUtils/particlesetdata.h \
    LocalizationUtils/RedSVD.h \
    LocalizationUtils/timestamp.h \
    LocalizationUtils/transform.h \
    LocalizationUtils/typeconverter.h \
    LocalizationUtils/vcudata.h \
    LocalizationUtils/velocalib.h \
    LocalizationUtils/velodyneringdata.h \
    VelodynePcapCapture.h \
    ECSegmentation.h \
    TrajCapture.h
