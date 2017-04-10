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

QMAKE_CXXFLAGS += -std=c++11 -ffast-math

QMAKE_CXXFLAGS_RELEASE -= -O2
QMAKE_CXXFLAGS_RELEASE += -O3

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
    opusaudioout.cpp \
    JSound.cpp \
    ../rtaudio-4.1.2/RtAudio.cpp \
    jconvolutionalcodec.cpp \
    dscadatadeformatter.cpp \
    ../kiss_fft130/kiss_fastfir_complex.c \
    ../kiss_fft130/kiss_fastfir_real.c \
    sdr.cpp

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
    opusaudioout.h \
    JSound.h \
    ../rtaudio-4.1.2/RtAudio.h \
    jconvolutionalcodec.h \
    dscadatadeformatter.h \
    ../kiss_fft130/kiss_fastfir_complex.h \
    ../kiss_fft130/kiss_fastfir_real.h \
    sdr.h


win32 {
#on windows this is where I put the headers of the 3rd party libraries that are linked to.
    HEADERS  += ../librtlsdr/include/rtl-sdr.h \
    ../librtlsdr/include/rtl-sdr_export.h \
    ../libcorrect/include/correct.h \
    ../libopus-1.2-alpha/include/opus/opus.h
#on windows i have no problem with 192k sample sound cards so I use 192k
    DEFINES += SAMPLE_RATE_IN=192000 \
        BUFFER_FRAMES=16384
}
unix {
# on the pi Qt creator doesn't seem know about /usr/include and /usr/local/include so I've added them here so it knows
    INCLUDEPATH += /usr/include \
    /usr/local/include
#for me they are saved here but you dont need this
    HEADERS  += /usr/include/rtl-sdr.h \
    /usr/include/rtl-sdr_export.h \
    /usr/local/include/correct.h \
    /usr/local/include/opus/opus/opus.h
#this is for the pi. srinking the window to fast on the pi seems to not work quite right and removes the menu bar
    DEFINES += __SLOW_SRINK_WINDOW__
#on the pi I havn't got 192k to work so I just use 48k in this case
    DEFINES += SAMPLE_RATE_IN=48000 \
        BUFFER_FRAMES=4096
}

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

#on windows you will have to build or obtain dlls for libsdr,libopus, and libcorrect

# here we have librtlsdr


win32 {
#on windows the libsdr dlls are here
INCLUDEPATH +=../librtlsdr/include
contains(QT_ARCH, i386) {
    #message("32-bit")

    LIBS += -L$$PWD/../librtlsdr/32
} else {
    #message("64-bit")
    LIBS += -L$$PWD/../librtlsdr/64
}

}

LIBS += -lrtlsdr

# here we have libcorrect


win32 {
#on windows the libcorrect dlls are here
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

unix {
LIBS += -lcorrect
}


# here we have libopus

win32 {
#on windows the libopus dlls are here
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

unix {
LIBS += -lopus
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
#Select your audio system. For me ALSA seems the best on the Raspberry Pi3, the other ones I get problems
unix {
#    DEFINES += __UNIX_JACK__
#    DEFINES += __LINUX_PULSE__
    DEFINES += __LINUX_ALSA__
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
