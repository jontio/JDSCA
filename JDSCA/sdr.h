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
    //various ones to choose from. its a complex filter so the bandwidth is 2* the passband
    //the constelations seem to colapse due to these things

    //Sampling Frequency : 1200000;
    //Passband Frequency : 85000;
    //Stopband Frequecncy : 105000;
    //Passband Ripple (dB) : 5.000000e-02;
    //Stopband Attenuation (dB) : 60;
//    cpx_type cx;cpx_type cy;
//    const double gain_1=6.958355120e-01;const double b0_1=1.000000000e+00;const double b1_1=-1.659940833e+00;const double b2_1=1.000000000e+00;const double a1_1=-1.719221713e+00;const double a2_1=8.911986451e-01;cpx_type cz0_1;cpx_type cz1_1;
//    const double gain_2=5.135450097e-01;const double b0_2=1.000000000e+00;const double b1_2=-1.420034466e+00;const double b2_2=1.000000000e+00;const double a1_2=-1.655916880e+00;const double a2_2=7.700013426e-01;cpx_type cz0_2;cpx_type cz1_2;
//    const double gain_3=3.457082444e-01;const double b0_3=1.000000000e+00;const double b1_3=1.576256538e-01;const double b2_3=1.000000000e+00;const double a1_3=-1.594989536e+00;const double a2_3=6.484798724e-01;cpx_type cz0_3;cpx_type cz1_3;
//    const double gain_4=1.415891309e-02;const double b0_4=1.000000000e+00;const double b1_4=-1.716918327e+00;const double b2_4=1.000000000e+00;const double a1_4=-1.767632786e+00;const double a2_4=9.695611149e-01;cpx_type cz0_4;cpx_type cz1_4;

    //Sampling Frequency : 1200000;
    //Passband Frequency : 95000;
    //Stopband Frequecncy : 120000;
    //Passband Ripple (dB) : 1.000000e-02;
    //Stopband Attenuation (dB) : 60;
//    cpx_type cx;cpx_type cy;
//    const double gain_1=6.527141816e-01;const double b0_1=1.000000000e+00;const double b1_1=-1.525931357e+00;const double b2_1=1.000000000e+00;const double a1_1=-1.619157064e+00;const double a2_1=8.421292387e-01;cpx_type cz0_1=0;cpx_type cz1_1=0;
//    const double gain_2=4.832953160e-01;const double b0_2=1.000000000e+00;const double b1_2=-1.189410591e+00;const double b2_2=1.000000000e+00;const double a1_2=-1.530932371e+00;const double a2_2=6.862616235e-01;cpx_type cz0_2=0;cpx_type cz1_2=0;
//    const double gain_3=3.993993060e-01;const double b0_3=1.000000000e+00;const double b1_3=5.760090221e-01;const double b2_3=1.000000000e+00;const double a1_3=-1.452176123e+00;const double a2_3=5.416669506e-01;cpx_type cz0_3=0;cpx_type cz1_3=0;
//    const double gain_4=1.668442256e-02;const double b0_4=1.000000000e+00;const double b1_4=-1.610219116e+00;const double b2_4=1.000000000e+00;const double a1_4=-1.691664787e+00;const double a2_4=9.536529972e-01;cpx_type cz0_4=0;cpx_type cz1_4=0;

    //this one is the best but the bandwidth is bigger than I thought it should be
    //Sampling Frequency : 1200000;
    //Passband Frequency : 120000;
    //Stopband Frequecncy : 200000;
    //Passband Ripple (dB) : 1.000000e-02;
    //Stopband Attenuation (dB) : 60;
//    cpx_type cx;cpx_type cy;
//    const double gain_1=3.826143438e-01;const double b0_1=1.000000000e+00;const double b1_1=-1.240293996e+00;const double b2_1=1.000000000e+00;const double a1_1=-1.487362522e+00;const double a2_1=9.118618836e-01;cpx_type cz0_1=0;cpx_type cz1_1=0;
//    const double gain_2=3.313614411e-01;const double b0_2=1.000000000e+00;const double b1_2=-1.006250344e+00;const double b2_2=1.000000000e+00;const double a1_2=-1.391253707e+00;const double a2_2=7.205440254e-01;cpx_type cz0_2=0;cpx_type cz1_2=0;
//    const double gain_3=1.032644160e-01;const double b0_3=1.000000000e+00;const double b1_3=-5.906077665e-02;const double b2_3=1.000000000e+00;const double a1_3=-1.309769599e+00;const double a2_3=5.101995539e-01;cpx_type cz0_3=0;cpx_type cz1_3=0;
//    const double gain_4=2.657856794e-01;const double b0_4=1.000000000e+00;const double b1_4=1.000000000e+00;const double b2_4=0.000000000e+00;const double a1_4=-6.360084201e-01;const double a2_4=0.000000000e+00;cpx_type cz0_4=0;cpx_type cz1_4=0;

    //Sampling Frequency : 1200000;
    //Passband Frequency : 100000;
    //Stopband Frequecncy : 120000;
    //Passband Ripple (dB) : 1.000000e-03;
    //Stopband Attenuation (dB) : 40;
//    cpx_type cx;cpx_type cy;
//    const double gain_1=7.824919130e-01;const double b0_1=1.000000000e+00;const double b1_1=-1.561117273e+00;const double b2_1=1.000000000e+00;const double a1_1=-1.557049441e+00;const double a2_1=8.379132903e-01;cpx_type cz0_1=0;cpx_type cz1_1=0;
//    const double gain_2=6.170967785e-01;const double b0_2=1.000000000e+00;const double b1_2=-1.275532644e+00;const double b2_2=1.000000000e+00;const double a1_2=-1.385150223e+00;const double a2_2=6.260288972e-01;cpx_type cz0_2=0;cpx_type cz1_2=0;
//    const double gain_3=5.785293654e-01;const double b0_3=1.000000000e+00;const double b1_3=4.029873378e-01;const double b2_3=1.000000000e+00;const double a1_3=-1.180816523e+00;const double a2_3=3.737010553e-01;cpx_type cz0_3=0;cpx_type cz1_3=0;
//    const double gain_4=4.991869068e-02;const double b0_4=1.000000000e+00;const double b1_4=-1.629789091e+00;const double b2_4=1.000000000e+00;const double a1_4=-1.654228892e+00;const double a2_4=9.565358453e-01;cpx_type cz0_4=0;cpx_type cz1_4=0;

    //bad
    //Sampling Frequency : 1200000;
    //Passband Frequency : 100000;
    //Stopband Frequecncy : 120000;
    //Passband Ripple (dB) : 5.000000e-02;
    //Stopband Attenuation (dB) : 60;
//    cpx_type cx;cpx_type cy;
//    const double gain_1=7.004982447e-01;const double b0_1=1.000000000e+00;const double b1_1=-1.538920606e+00;const double b2_1=1.000000000e+00;const double a1_1=-1.639844068e+00;const double a2_1=8.745866535e-01;cpx_type cz0_1=0;cpx_type cz1_1=0;
//    const double gain_2=5.263113548e-01;const double b0_2=1.000000000e+00;const double b1_2=-1.231562239e+00;const double b2_2=1.000000000e+00;const double a1_2=-1.581338917e+00;const double a2_2=7.362552615e-01;cpx_type cz0_2=0;cpx_type cz1_2=0;
//    const double gain_3=4.054145631e-01;const double b0_3=1.000000000e+00;const double b1_3=4.861027861e-01;const double b2_3=1.000000000e+00;const double a1_3=-1.526605177e+00;const double a2_3=5.988834759e-01;cpx_type cz0_3=0;cpx_type cz1_3=0;
//    const double gain_4=1.423306342e-02;const double b0_4=1.000000000e+00;const double b1_4=-1.614036898e+00;const double b2_4=1.000000000e+00;const double a1_4=-1.688011757e+00;const double a2_4=9.647694835e-01;cpx_type cz0_4=0;cpx_type cz1_4=0;


    //Sampling Frequency : 1200000;
    //Passband Frequency : 85000;
    //Stopband Frequecncy : 120000;
    //Passband Ripple (dB) : 5.000000e-02;
    //Stopband Attenuation (dB) : 60;
//    cpx_type cx;cpx_type cy;
//    const double gain_1=3.162408194e-01;const double b0_1=1.000000000e+00;const double b1_1=-1.647429466e+00;const double b2_1=1.000000000e+00;const double a1_1=-1.744746957e+00;const double a2_1=9.510380955e-01;cpx_type cz0_1=0;cpx_type cz1_1=0;
//    const double gain_2=3.412271383e-01;const double b0_2=1.000000000e+00;const double b1_2=-1.529730272e+00;const double b2_2=1.000000000e+00;const double a1_2=-1.674108330e+00;const double a2_2=8.345771230e-01;cpx_type cz0_2=0;cpx_type cz1_2=0;
//    const double gain_3=8.226397084e-02;const double b0_3=1.000000000e+00;const double b1_3=-9.246095328e-01;const double b2_3=1.000000000e+00;const double a1_3=-1.601585991e+00;const double a2_3=6.900518807e-01;cpx_type cz0_3=0;cpx_type cz1_3=0;
//    const double gain_4=2.016190514e-01;const double b0_4=1.000000000e+00;const double b1_4=1.000000000e+00;const double b2_4=0.000000000e+00;const double a1_4=-7.820559935e-01;const double a2_4=0.000000000e+00;cpx_type cz0_4=0;cpx_type cz1_4=0;

    //Sampling Frequency : 1200000;
    //Passband Frequency : 85000;
    //Stopband Frequecncy : 120000;
    //Passband Ripple (dB) : 1.000000e-03;
    //Stopband Attenuation (dB) : 60;
//    cpx_type cx;cpx_type cy;
//    const double gain_1=5.924226432e-01;const double b0_1=1.000000000e+00;const double b1_1=-1.532467557e+00;const double b2_1=1.000000000e+00;const double a1_1=-1.605731944e+00;const double a2_1=8.064119693e-01;cpx_type cz0_1=0;cpx_type cz1_1=0;
//    const double gain_2=4.370423940e-01;const double b0_2=1.000000000e+00;const double b1_2=-1.173292816e+00;const double b2_2=1.000000000e+00;const double a1_2=-1.484398667e+00;const double a2_2=6.365776884e-01;cpx_type cz0_2=0;cpx_type cz1_2=0;
//    const double gain_3=3.918739357e-01;const double b0_3=1.000000000e+00;const double b1_3=6.287999016e-01;const double b2_3=1.000000000e+00;const double a1_3=-1.381519761e+00;const double a2_3=4.894225783e-01;cpx_type cz0_3=0;cpx_type cz1_3=0;
//    const double gain_4=1.973782718e-02;const double b0_4=1.000000000e+00;const double b1_4=-1.624239712e+00;const double b2_4=1.000000000e+00;const double a1_4=-1.708433704e+00;const double a2_4=9.404887921e-01;cpx_type cz0_4=0;cpx_type cz1_4=0;

    //ok
    //Sampling Frequency : 1200000;
    //Passband Frequency : 120000;
    //Stopband Frequecncy : 150000;
    //Passband Ripple (dB) : 2.000000e-02;
    //Stopband Attenuation (dB) : 60;
    cpx_type cx;cpx_type cy;
    const double gain_1=6.831108554e-01;const double b0_1=1.000000000e+00;const double b1_1=-1.313855798e+00;const double b2_1=1.000000000e+00;const double a1_1=-1.489941758e+00;const double a2_1=8.282912918e-01;cpx_type cz0_1=0;cpx_type cz1_1=0;
    const double gain_2=5.253787637e-01;const double b0_2=1.000000000e+00;const double b1_2=-8.863998696e-01;const double b2_2=1.000000000e+00;const double a1_2=-1.425537513e+00;const double a2_2=6.537632118e-01;cpx_type cz0_2=0;cpx_type cz1_2=0;
    const double gain_3=5.098542519e-01;const double b0_3=1.000000000e+00;const double b1_3=9.226566841e-01;const double b2_3=1.000000000e+00;const double a1_3=-1.369560142e+00;const double a2_3=4.897449865e-01;cpx_type cz0_3=0;cpx_type cz1_3=0;
    const double gain_4=1.582646632e-02;const double b0_4=1.000000000e+00;const double b1_4=-1.425576003e+00;const double b2_4=1.000000000e+00;const double a1_4=-1.548928949e+00;const double a2_4=9.501363359e-01;cpx_type cz0_4=0;cpx_type cz1_4=0;



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
