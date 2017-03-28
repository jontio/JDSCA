#-------------------------------------------------
#
# Project created by QtCreator 2015-08-01T20:41:23
# Jonti Olds
#
#-------------------------------------------------

QT       += core gui svg

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets  printsupport

TARGET = JDSCA
TEMPLATE = app

#message("QT_ARCH is \"$$QT_ARCH\"");
contains(QT_ARCH, i386) {
    #message("32-bit")
    DEFINES += kiss_fft_scalar=float
} else {
    #message("64-bit")
    DEFINES += kiss_fft_scalar=double
}

SOURCES += main.cpp\
        mainwindow.cpp \
    coarsefreqestimate.cpp \
    DSP.cpp \
    ../qcustomplot/qcustomplot.cpp \
    gui_classes/console.cpp \
    gui_classes/qscatterplot.cpp \
    gui_classes/qspectrumdisplay.cpp \
    gui_classes/qled.cpp \
    ../kiss_fft130/kiss_fft.c \
    fftwrapper.cpp \
    fftrwrapper.cpp \
    ../kiss_fft130/kiss_fftr.c \
    gui_classes/settingsdialog.cpp \
    oqpskdemodulator.cpp \
    ../kiss_fft130/kiss_fastfir.c \
    opusaudioout.cpp \
    JSound.cpp \
    ../rtaudio-4.1.2/RtAudio.cpp \
    jconvolutionalcodec.cpp \
    dscadatadeformatter.cpp

HEADERS  += mainwindow.h \
    coarsefreqestimate.h \
    DSP.h \
    ../qcustomplot/qcustomplot.h \
    gui_classes/console.h \
    gui_classes/qscatterplot.h \
    gui_classes/qspectrumdisplay.h \
    gui_classes/qled.h \
    ../kiss_fft130/_kiss_fft_guts.h \
    ../kiss_fft130/kiss_fft.h \
    fftwrapper.h \
    fftrwrapper.h \
    ../kiss_fft130/kiss_fftr.h \
    gui_classes/settingsdialog.h \
    oqpskdemodulator.h \
    ../kiss_fft130/kiss_fastfir.h \
    ../libopus-1.2-alpha/include/opus/opus.h \
    opusaudioout.h \
    JSound.h \
    ../rtaudio-4.1.2/RtAudio.h \
    ../libcorrect/include/correct.h \
    jconvolutionalcodec.h \
    dscadatadeformatter.h

FORMS    += mainwindow.ui \
    gui_classes/settingsdialog.ui

RESOURCES += \
    jdsca.qrc

DISTFILES += \
    ../kiss_fft130/TIPS \
    ../kiss_fft130/CHANGELOG \
    ../kiss_fft130/COPYING \
    ../kiss_fft130/README \
    ../qcustomplot/changelog.txt \
    ../qcustomplot/GPL.txt \
    ../README.md \
    ../images/JDSCA.png

win32 {
RC_FILE = jdsca.rc
}

# here we have libcorrect


win32 {

INCLUDEPATH +=../libcorrect/include

contains(QT_ARCH, i386) {
    #message("32-bit")

    LIBS += -L$$PWD/../libcorrect/lib/32
} else {
    #message("64-bit")
    LIBS += -L$$PWD/../libcorrect/lib/64
}

    LIBS += -llibcorrect


}

# here we have libopus

win32 {

INCLUDEPATH +=../libopus-1.2-alpha/include
contains(QT_ARCH, i386) {
    #message("32-bit")
    LIBS += -L$$PWD/../libopus-1.2-alpha/lib/32
} else {
    #message("64-bit")
    LIBS += -L$$PWD/../libopus-1.2-alpha/lib/64
}

    LIBS += -llibopus

}

# from here on this is for rtaudio if we include it


INCLUDEPATH += ../rtaudio-4.1.2/include

#You dont need all these audio APIs.
#Say if you want only jack then remove __LINUX_PULSE__ and __LINUX_ALSA__ leaving just DEFINES += __UNIX_JACK__
#I have never compiled on mac so if you have let me know if anything should be changed
win32 {
    DEFINES += __WINDOWS_DS__
    #  not sure what the advantages for WASAPI is over DS DEFINES += __WINDOWS_WASAPI__


}
unix {
    DEFINES += __UNIX_JACK__ \
            __LINUX_PULSE__ \
            __LINUX_ALSA__
}
macx {
    DEFINES += __MACOSX_CORE__ \
            __UNIX_JACK__
}

contains(DEFINES, __WINDOWS_DS__) {
    LIBS += libdsound libole32 libwinmm
}

contains(DEFINES, __WINDOWS_WASAPI__) {
    LIBS += libole32 libwinmm libksuser libuuid
}

contains(DEFINES, __UNIX_JACK__) {
    LIBS += -ljack -lpthread
}

contains(DEFINES, __LINUX_PULSE__) {
    LIBS += -lpthread -lpulse-simple -lpulse
}

contains(DEFINES, __LINUX_ALSA__ ) {
    LIBS += -lasound -lpthread
}

#are these the right things to link with for coreaudio?
contains(DEFINES, __MACOSX_CORE__ ) {
    LIBS += -lCoreAudio -lpthread
}

unix {
    target.path = /usr/lib
    INSTALLS += target
}
