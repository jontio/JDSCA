#ifndef SDR_H
#define SDR_H

#include <QObject>
#include "DSP.h"
#include <QtConcurrent/QtConcurrentRun>
#include <rtl-sdr.h>
#include <QVector>
#include <QMutex>
#include <unistd.h>
#include <QWaitCondition>

class SDR_Enum
{
public:
    static QStringList DeviceNames()
    {
        //fill Rtl device list
        QStringList devices_names;
        int devcnt=rtlsdr_get_device_count();
        for(int i=0;i<devcnt;i++)devices_names<<QString::fromLocal8Bit(rtlsdr_get_device_name(i));
        return devices_names;
    }
};

class SDR : public QObject
{
Q_OBJECT
public:
    SDR(QObject *parent = 0);
    ~SDR();
    QStringList GetDeviceNames();
    bool StopAndCloseRtl();
    bool OpenRtl(int device_index);
    void SetGain(int dB);
    void SetAGC(bool enable);
    void SetAFC(bool enable);
    void SetFrequency(int freq);
    void SetSubCarrierFrequencyOffset(int freq);
    int GetSubCarrierFrequencyOffset();
    int GetFrequency();
    void StartRtl();

    rtlsdr_dev_t *rtldev;
    QVector<int> gains;
    quint32 freq_rtl;

    bool active;

signals:

    void audio_signal_out(const double *inputBuffer, qint64 nBufferFrames, int channels);

private slots:

private:
    int SubCarrierFrequencyOffset;
    bool StopRtl();

    //thread to send audio to demodulator
    bool demod_dispatcher();

    //thread to process the data from the RTL dongle
    void rtlsdr_callback(unsigned char *buf, uint32_t len);
    static void rtlsdr_callback_dispatcher(unsigned char *buf, uint32_t len, void *ctx)
    {
        SDR *inst=(SDR*)ctx;
        inst->rtlsdr_callback(buf,len);
    }

    //how we talk to the demod_dispatcher thread
    QFuture<bool> future_demod_dispatcher;
    bool do_demod_dispatcher_cancel;

    //how we talk to the rtl_callack thread
    QFuture<int> future_rtlsdr_callback;
    // also rtlsdr_cancel_async for stopping

    //For returning data from the rtl_callack thread to the demod_dispatcher thread
    #define N_BUFFERS 3
    QWaitCondition buffers_not_empty;
    QMutex buffers_mut;
    std::vector<double> buffers[N_BUFFERS];
    int buffers_size_valid[N_BUFFERS];
    int buffers_head_ptr=0;
    int buffers_tail_ptr=0;
    int buffers_used=0;

    //biquad BPF for FM signal (IF BPF)
    //this is better for the higher frequencies and goes up to 85kHz
    //Sampling Frequency : 1200000;
    //Passband Frequency : 85000;
    //Stopband Frequecncy : 105000;
    //Passband Ripple (dB) : 5.000000e-02;
    //Stopband Attenuation (dB) : 60;
    cpx_type cx;cpx_type cy;
    const double gain_1=6.958355120e-01;const double b0_1=1.000000000e+00;const double b1_1=-1.659940833e+00;const double b2_1=1.000000000e+00;const double a1_1=-1.719221713e+00;const double a2_1=8.911986451e-01;cpx_type cz0_1;cpx_type cz1_1;
    const double gain_2=5.135450097e-01;const double b0_2=1.000000000e+00;const double b1_2=-1.420034466e+00;const double b2_2=1.000000000e+00;const double a1_2=-1.655916880e+00;const double a2_2=7.700013426e-01;cpx_type cz0_2;cpx_type cz1_2;
    const double gain_3=3.457082444e-01;const double b0_3=1.000000000e+00;const double b1_3=1.576256538e-01;const double b2_3=1.000000000e+00;const double a1_3=-1.594989536e+00;const double a2_3=6.484798724e-01;cpx_type cz0_3;cpx_type cz1_3;
    const double gain_4=1.415891309e-02;const double b0_4=1.000000000e+00;const double b1_4=-1.716918327e+00;const double b2_4=1.000000000e+00;const double a1_4=-1.767632786e+00;const double a2_4=9.695611149e-01;cpx_type cz0_4;cpx_type cz1_4;

    //FM demod
    double zt0,zt1;
    double zb0,zb1;
    QMutex mut;
    std::vector<cpx_type> cblock3;
    WaveTableComplex *mixer;
    unsigned char peak_input_val;
    bool useafc;

    //frequency dropper for subcarrier signal
    QJHilbertFilter *hfir;
    WaveTableComplex *mixer3;

};

#endif // SDR_H
