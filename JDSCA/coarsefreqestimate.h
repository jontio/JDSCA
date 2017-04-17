#ifndef COARSEFREQESTIMATE_H
#define COARSEFREQESTIMATE_H

#include <QObject>
#include <QVector>

#include "DSP.h"

#include "fftwrapper.h"

#include <QFile>
#include <QDataStream>

typedef FFTWrapper<double> FFT;
typedef std::complex<double> cpx_type;

class CoarseFreqEstimate : public QObject
{
    Q_OBJECT
public:
    explicit CoarseFreqEstimate(QObject *parent = 0);
    ~CoarseFreqEstimate();
    void setSettings(int coarsefreqest_fft_power,double lockingbw,double fb,double Fs);
    void bigchange();
    void resettrycount();
signals:
    void FreqOffsetEstimate(double freq_offset_est);
    void BitrateEstimate(double bitrate_est);//this class also estimates bitrate you need to set vail bit rates in constructor
public slots:
    void ProcessBasebandData(const QVector<cpx_type> &data);
private:
    FFT *fft;
    FFT *ifft;
    QVector<cpx_type> out;
    QVector<cpx_type> in;
    double nfft;
    double Fs;
    int coarsefreqest_fft_power;
    double hzperbin;
    int startbin,stopbin;
    double lockingbw;
    double fb;
    double freq_offset_est;
    int emptyingcountdown;

    BitRateSearch bitratesearch;

    //debug
    //QVector<double> debug_buffer;
    //QVector<double> debug_buffer2;
    //int debug_buffer2_ptr;

    WavletFilter wavletfilter;


};

#endif // COARSEFREQESTIMATE_H
