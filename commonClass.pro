QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

include($$PWD/commonMath/commonMath.pri)
INCLUDEPATH += $$PWD/commonMath/

include($$PWD/commonGeometry/commonGeometry.pri)
INCLUDEPATH += $$PWD/commonGeometry/

include($$PWD/commonAlgo/commonAlgo.pri)
INCLUDEPATH += $$PWD/commonAlgo/

HEADER_FILES = $$files($$PWD/*.h, false)
SOURCE_FILES = $$files($$PWD/*.cpp, false)
RESOURCE_FILES = $$files($$PWD/*qrc, false)

HEADERS += $${HEADER_FILES}
SOURCES += $${SOURCE_FILES}
RESOURCES += $${RESOURCE_FILES}

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
