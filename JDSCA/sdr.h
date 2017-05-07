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

    typedef enum Filter_selection{FILTER_100_120=0,FILTER_120_150=1,FILTER_150_200=2,FILTER_200_300=3,FILTER_NONE}Filter_selection;
    typedef enum Discrim_selection{DISCRIM_APPROX,DISCRIM_ATAN2,DISCRIM_ATAN2_FAST}Discrim_selection;

    SDR(QObject *parent = 0);
    ~SDR();
    QStringList GetDeviceNames();
    bool StopAndCloseRtl();
    bool OpenRtl(int device_index);
    void SetGain(int dB);
    void SetAGC(bool enable);
    void SetAFC(bool enable);
    void SetFilterSelection(SDR::Filter_selection filter_selection);
    SDR::Filter_selection GetFilterSelection();
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

    cpx_type cx,cy;

    //Sampling Frequency : 1200000;
    //Passband Frequency : 100000;
    //Stopband Frequecncy : 120000;
    //Passband Ripple (dB) : 5.000000e-02;
    //Stopband Attenuation (dB) : 60;
    #ifndef CPX_TYPE
    typedef std::complex<double> cpx_type;
    #define CPX_TYPE
    #endif
    cpx_type cx_100_120k_BiQuad=0;cpx_type cy_100_120k_BiQuad=0;
    const double gain_100_120k_BiQuad_1=7.004982447e-01;const double b0_100_120k_BiQuad_1=1.000000000e+00;const double b1_100_120k_BiQuad_1=-1.538920606e+00;const double b2_100_120k_BiQuad_1=1.000000000e+00;const double a1_100_120k_BiQuad_1=-1.639844068e+00;const double a2_100_120k_BiQuad_1=8.745866535e-01;cpx_type cz0_100_120k_BiQuad_1=0;cpx_type cz1_100_120k_BiQuad_1=0;
    const double gain_100_120k_BiQuad_2=5.263113548e-01;const double b0_100_120k_BiQuad_2=1.000000000e+00;const double b1_100_120k_BiQuad_2=-1.231562239e+00;const double b2_100_120k_BiQuad_2=1.000000000e+00;const double a1_100_120k_BiQuad_2=-1.581338917e+00;const double a2_100_120k_BiQuad_2=7.362552615e-01;cpx_type cz0_100_120k_BiQuad_2=0;cpx_type cz1_100_120k_BiQuad_2=0;
    const double gain_100_120k_BiQuad_3=4.054145631e-01;const double b0_100_120k_BiQuad_3=1.000000000e+00;const double b1_100_120k_BiQuad_3=4.861027861e-01;const double b2_100_120k_BiQuad_3=1.000000000e+00;const double a1_100_120k_BiQuad_3=-1.526605177e+00;const double a2_100_120k_BiQuad_3=5.988834759e-01;cpx_type cz0_100_120k_BiQuad_3=0;cpx_type cz1_100_120k_BiQuad_3=0;
    const double gain_100_120k_BiQuad_4=1.423306342e-02;const double b0_100_120k_BiQuad_4=1.000000000e+00;const double b1_100_120k_BiQuad_4=-1.614036898e+00;const double b2_100_120k_BiQuad_4=1.000000000e+00;const double a1_100_120k_BiQuad_4=-1.688011757e+00;const double a2_100_120k_BiQuad_4=9.647694835e-01;cpx_type cz0_100_120k_BiQuad_4=0;cpx_type cz1_100_120k_BiQuad_4=0;


    //Sampling Frequency : 1200000;
    //Passband Frequency : 120000;
    //Stopband Frequecncy : 150000;
    //Passband Ripple (dB) : 2.000000e-02;
    //Stopband Attenuation (dB) : 60;
    #ifndef CPX_TYPE
    typedef std::complex<double> cpx_type;
    #define CPX_TYPE
    #endif
    cpx_type cx_120_150k_BiQuad=0;cpx_type cy_120_150k_BiQuad=0;
    const double gain_120_150k_BiQuad_1=6.831108554e-01;const double b0_120_150k_BiQuad_1=1.000000000e+00;const double b1_120_150k_BiQuad_1=-1.313855798e+00;const double b2_120_150k_BiQuad_1=1.000000000e+00;const double a1_120_150k_BiQuad_1=-1.489941758e+00;const double a2_120_150k_BiQuad_1=8.282912918e-01;cpx_type cz0_120_150k_BiQuad_1=0;cpx_type cz1_120_150k_BiQuad_1=0;
    const double gain_120_150k_BiQuad_2=5.253787637e-01;const double b0_120_150k_BiQuad_2=1.000000000e+00;const double b1_120_150k_BiQuad_2=-8.863998696e-01;const double b2_120_150k_BiQuad_2=1.000000000e+00;const double a1_120_150k_BiQuad_2=-1.425537513e+00;const double a2_120_150k_BiQuad_2=6.537632118e-01;cpx_type cz0_120_150k_BiQuad_2=0;cpx_type cz1_120_150k_BiQuad_2=0;
    const double gain_120_150k_BiQuad_3=5.098542519e-01;const double b0_120_150k_BiQuad_3=1.000000000e+00;const double b1_120_150k_BiQuad_3=9.226566841e-01;const double b2_120_150k_BiQuad_3=1.000000000e+00;const double a1_120_150k_BiQuad_3=-1.369560142e+00;const double a2_120_150k_BiQuad_3=4.897449865e-01;cpx_type cz0_120_150k_BiQuad_3=0;cpx_type cz1_120_150k_BiQuad_3=0;
    const double gain_120_150k_BiQuad_4=1.582646632e-02;const double b0_120_150k_BiQuad_4=1.000000000e+00;const double b1_120_150k_BiQuad_4=-1.425576003e+00;const double b2_120_150k_BiQuad_4=1.000000000e+00;const double a1_120_150k_BiQuad_4=-1.548928949e+00;const double a2_120_150k_BiQuad_4=9.501363359e-01;cpx_type cz0_120_150k_BiQuad_4=0;cpx_type cz1_120_150k_BiQuad_4=0;

    //Sampling Frequency : 1200000;
    //Passband Frequency : 150000;
    //Stopband Frequecncy : 200000;
    //Passband Ripple (dB) : 5.000000e-03;
    //Stopband Attenuation (dB) : 60;
    #ifndef CPX_TYPE
    typedef std::complex<double> cpx_type;
    #define CPX_TYPE
    #endif
    cpx_type cx_150_200k_BiQuad=0;cpx_type cy_150_200k_BiQuad=0;
    const double gain_150_200k_BiQuad_1=6.660802832e-01;const double b0_150_200k_BiQuad_1=1.000000000e+00;const double b1_150_200k_BiQuad_1=-8.968007894e-01;const double b2_150_200k_BiQuad_1=1.000000000e+00;const double a1_150_200k_BiQuad_1=-1.223331318e+00;const double a2_150_200k_BiQuad_1=7.529173665e-01;cpx_type cz0_150_200k_BiQuad_1=0;cpx_type cz1_150_200k_BiQuad_1=0;
    const double gain_150_200k_BiQuad_2=5.432292693e-01;const double b0_150_200k_BiQuad_2=1.000000000e+00;const double b1_150_200k_BiQuad_2=-3.127891487e-01;const double b2_150_200k_BiQuad_2=1.000000000e+00;const double a1_150_200k_BiQuad_2=-1.159558964e+00;const double a2_150_200k_BiQuad_2=5.294187106e-01;cpx_type cz0_150_200k_BiQuad_2=0;cpx_type cz1_150_200k_BiQuad_2=0;
    const double gain_150_200k_BiQuad_3=7.202922772e-01;const double b0_150_200k_BiQuad_3=1.000000000e+00;const double b1_150_200k_BiQuad_3=1.360062974e+00;const double b2_150_200k_BiQuad_3=1.000000000e+00;const double a1_150_200k_BiQuad_3=-1.109981918e+00;const double a2_150_200k_BiQuad_3=3.347398786e-01;cpx_type cz0_150_200k_BiQuad_3=0;cpx_type cz1_150_200k_BiQuad_3=0;
    const double gain_150_200k_BiQuad_4=1.835159808e-02;const double b0_150_200k_BiQuad_4=1.000000000e+00;const double b1_150_200k_BiQuad_4=-1.068964572e+00;const double b2_150_200k_BiQuad_4=1.000000000e+00;const double a1_150_200k_BiQuad_4=-1.291784521e+00;const double a2_150_200k_BiQuad_4=9.247654531e-01;cpx_type cz0_150_200k_BiQuad_4=0;cpx_type cz1_150_200k_BiQuad_4=0;

    //Sampling Frequency : 1200000;
    //Passband Frequency : 200000;
    //Stopband Frequecncy : 300000;
    //Passband Ripple (dB) : 2.000000e-02;
    //Stopband Attenuation (dB) : 60;
    #ifndef CPX_TYPE
    typedef std::complex<double> cpx_type;
    #define CPX_TYPE
    #endif
    cpx_type cx_200_300k_BiQuad=0;cpx_type cy_200_300k_BiQuad=0;
    const double gain_200_300k_BiQuad_1=6.233821143e-01;const double b0_200_300k_BiQuad_1=1.000000000e+00;const double b1_200_300k_BiQuad_1=5.672813696e-01;const double b2_200_300k_BiQuad_1=1.000000000e+00;const double a1_200_300k_BiQuad_1=-7.861964934e-01;const double a2_200_300k_BiQuad_1=4.991804423e-01;cpx_type cz0_200_300k_BiQuad_1=0;cpx_type cz1_200_300k_BiQuad_1=0;
    const double gain_200_300k_BiQuad_2=1.203129272e+00;const double b0_200_300k_BiQuad_2=1.000000000e+00;const double b1_200_300k_BiQuad_2=1.697850380e+00;const double b2_200_300k_BiQuad_2=1.000000000e+00;const double a1_200_300k_BiQuad_2=-8.562041022e-01;const double a2_200_300k_BiQuad_2=2.317694521e-01;cpx_type cz0_200_300k_BiQuad_2=0;cpx_type cz1_200_300k_BiQuad_2=0;
    const double gain_200_300k_BiQuad_3=1.949144472e-02;const double b0_200_300k_BiQuad_3=1.000000000e+00;const double b1_200_300k_BiQuad_3=4.428932832e-02;const double b2_200_300k_BiQuad_3=1.000000000e+00;const double a1_200_300k_BiQuad_3=-7.658540886e-01;const double a2_200_300k_BiQuad_3=8.278199534e-01;cpx_type cz0_200_300k_BiQuad_3=0;cpx_type cz1_200_300k_BiQuad_3=0;

    Filter_selection filter_selection=FILTER_200_300;
    Discrim_selection discrim_selection=DISCRIM_ATAN2_FAST;//this seems ok. it's close to atan2

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
