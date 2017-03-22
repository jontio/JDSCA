#ifndef OQPSKDEMODULATOR_H
#define OQPSKDEMODULATOR_H

#include <QIODevice>
#include "DSP.h"
#include <QVector>
#include <complex>
#include <QPointer>
#include <QObject>
#include <QElapsedTimer>
#include "coarsefreqestimate.h"
#include <assert.h>
#include "dscadatadeformatter.h"
#include <QMutex>

#include <QFile>

class OqpskDemodulator : public QIODevice
{
    Q_OBJECT
public:
    enum ScatterPointType{ SPT_constellation, SPT_phaseoffseterror, SPT_phaseoffsetest,SPT_8constellation, SPT_None};
    struct Settings
    {
        int coarsefreqest_fft_power;
        double freq_center;
        double lockingbw;
        double fb;
        double Fs;
        double signalthreshold;
        Settings()
        {
            coarsefreqest_fft_power=16;//2^coarsefreqest_fft_power
            freq_center=8000;//Hz
            fb=12500;//bps
            Fs=48000;//Hz
            signalthreshold=0.5;
            lockingbw=10500;
        }
    };
    explicit OqpskDemodulator(QObject *parent);
    ~OqpskDemodulator();
    void setBandwidth(double bandwidth_hz);
    double getBandwidth(){return lockingbw;}
    void setCMA(bool enable){cma_enabled=enable;cma.reset();}
    bool getCMA(){return cma_enabled;}
    void setAFC(bool state);
    void setSettings(Settings settings);
    void ConnectSinkDevice(QIODevice *datasinkdevice);
    void DisconnectSinkDevice();
    void start();
    void stop();
    qint64 readData(char *data, qint64 maxlen);
    qint64 writeData(const char *data, qint64 len);
    double getCurrentFreq();
    void setScatterPointType(ScatterPointType type);
signals:
    void ScatterPoints(const QVector<cpx_type> &buffer);
    void OrgOverlapedBuffer(const QVector<double> &buffer);
    void PeakVolume(double Volume);
    void SampleRateChanged(double Fs);
    void BitRateChanged(double fb);
    void Plottables(double freq_est,double freq_center,double bandwidth);
    void BBOverlapedBuffer(const QVector<cpx_type> &buffer);
    void MSESignal(double mse);
    void SignalStatus(bool gotasignal);
    void WarningTextSignal(const QString &str);
    void EbNoMeasurmentSignal(double EbNo);
private:
    QPointer<QIODevice> pdatasinkdevice;
    bool afc;
    int scatterpointtype;

    QVector<double> spectrumcycbuff;
    int spectrumcycbuff_ptr;
    int spectrumnfft;

    QVector<cpx_type> bbcycbuff;
    int bbcycbuff_ptr;
    int bbnfft;

    QVector<cpx_type> pointbuff;
    int pointbuff_ptr;

    QElapsedTimer timer;

    bool cma_enabled;

    double Fs;
    double freq_center;
    double lockingbw;
    double fb;
    double signalthreshold;

    double SamplesPerSymbol;

    FIR *fir_re;
    FIR *fir_im;

    //st
    Delay<double> delays;
    Delay<double> delayt41;
    Delay<double> delayt42;
    Delay<double> delayt8;
    IIR st_iir_resonator;
    WaveTable st_osc;
    WaveTable st_osc_ref;

    double st_timeing_error_systematic;

    //ct
    IIR ct_iir_loopfilter;

    WaveTable mixer_center;
    WaveTable mixer2;

    CoarseFreqEstimate *coarsefreqestimate;


    double mse;
    MSEcalc *msecalc;

    QVector<cpx_type> phasepointbuff;
    int phasepointbuff_ptr;

    AGC *agc;

    MERMeasure *mermeasure;

    BaceConverter bc;
    QByteArray  RxDataBits;//packed in bytes

    MovingAverage *marg;
    DelayThing<cpx_type> dt;


    QJFastFIRFilter *bluebpf;
    QTimer *bpfupdatetimer;

    CMA cma;


    bool dcd;

    bool freqestimator_working;

    Settings settings;

public slots:
    void BitrateEstimate(double bitrate_est);
    void FreqOffsetEstimateSlot(double freq_offset_est);
    void CenterFreqChangedSlot(double freq_center);
    void DCDstatSlot(bool dcd);
private slots:
    void setBandPassFilter();
};

#endif // OQPSKDEMODULATOR_H
