#-------------------------------------------------
#
# Project created by QtCreator 2015-12-06T15:12:54
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PR2_Interface
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    sensorvisualizer.cpp \
    connectioninfo.cpp \
    widget.cpp \
    pr2robotvisualizer.cpp \
    lifetimestatistics.cpp \
    datagraphs.cpp \
    programlist.cpp \
    pulsedetection.cpp \
    liftobject.cpp \
    placeobject.cpp \
    swaphands.cpp \
    rqtdockingwindow.cpp \
    rotateobject.cpp

HEADERS  += mainwindow.h \
    sensorvisualizer.h \
    connectioninfo.h \
    widget.h \
    pr2robotvisualizer.h \
    lifetimestatistics.h \
    datagraphs.h \
    programlist.h \
    pulsedetection.h \
    liftobject.h \
    placeobject.h \
    swaphands.h \
    rqtdockingwindow.h \
    rotateobject.h

FORMS    += mainwindow.ui \
    sensorvisualizer.ui \
    connectioninfo.ui \
    Index.ui \
    pr2robotvisualizer.ui \
    lifetimestatistics.ui \
    datagraphs.ui \
    programlist.ui \
    pulsedetection.ui \
    liftobject.ui \
    placeobject.ui \
    swaphands.ui \
    rqtdockingwindow.ui \
    rotateobject.ui

DISTFILES +=
