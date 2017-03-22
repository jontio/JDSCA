#include "coarsefreqestimate.h"
#include <assert.h>
#include <QDebug>

CoarseFreqEstimate::CoarseFreqEstimate(QObject *parent) : QObject(parent)
{
    coarsefreqest_fft_power=13;
    lockingbw=500;//Hz
    fb=125;
    Fs=8000;//Hz

    nfft=pow(2,coarsefreqest_fft_power);
    fft = new FFT(nfft,false);
    ifft = new FFT(nfft,true);
    hzperbin=Fs/((double)nfft);
    out.resize(nfft);
    in.resize(nfft);
    startbin=round(lockingbw/hzperbin);
    stopbin=nfft-startbin;
    emptyingcountdown=1;

    bitratesearch.setSetting(Fs,0.8,4,100,10,3.0,1.0,lockingbw);

}

void CoarseFreqEstimate::setSettings(int _coarsefreqest_fft_power,double _lockingbw,double _fb,double _Fs)
{
    coarsefreqest_fft_power=_coarsefreqest_fft_power;
    lockingbw=_lockingbw;//Hz
    fb=_fb;
    Fs=_Fs;//Hz

    double tnfft=pow(2,coarsefreqest_fft_power);
    if(tnfft!=nfft)
    {
        nfft=tnfft;
        delete ifft;
        delete fft;
        fft = new FFT(nfft,false);
        ifft = new FFT(nfft,true);
    }
    hzperbin=Fs/((double)nfft);
    out.resize(nfft);
    in.resize(nfft);
    startbin=std::max(round(lockingbw/hzperbin),1.0);
    stopbin=nfft-startbin;

    //bit rate detection settings.
    bitratesearch.validbitrates.clear();
    bitratesearch.validbitrates.push_back(10500);
    bitratesearch.validbitrates.push_back(19000);
    bitratesearch.validbitrates.push_back(24000);
    bitratesearch.validbitrates.push_back(32000);
    bitratesearch.validbitrates.push_back(42000);
    bitratesearch.validbitrates.push_back(50000);
    bitratesearch.validbitrates.push_back(56000);
    bitratesearch.validbitrates.push_back(72000);
    bitratesearch.validbitrates.push_back(82000);
    bitratesearch.validbitrates.push_back(94000);
    bitratesearch.setSetting(Fs,0.2,10,10000.0/hzperbin,10,100.0,0.1,lockingbw);//smoothing has problems when freq is drifting so better to keep small i think
    //bitratesearch.setSetting(Fs,0.1,10,10000.0/hzperbin,10,100.0,1.0,lockingbw);
}

CoarseFreqEstimate::~CoarseFreqEstimate()
{

    delete ifft;
    delete fft;
}

void CoarseFreqEstimate::bigchange()
{
    emptyingcountdown=4;
}

void CoarseFreqEstimate::ProcessBasebandData(const QVector<cpx_type> &data)
{
    //fft size must be even. 2^n is best
    assert(nfft==data.size());

    //remove high frequencies then square and do fft and shift (0hz bin is at nfft/2)
    fft->transform(data,out);
    for(int i=startbin;i<=stopbin;i++)out[i]=0;
    ifft->transform(out,in);
    for(int i=0;i<nfft;i++)in[i]=in[i]*in[i];
    fft->transform(in,out);

    for(int i=0;i<nfft/2;i++)std::swap(out[i+nfft/2],out[i]);

    //find peaks to find the symbol rate, we could also estimate the fgrequency here but we dont. We probably should as this is a better algo
    bitratesearch.update(out);

    //notify anyone interest that we belive the bit rate has changed
    //might as well change our estimated fold location too
    if((fb!=bitratesearch.rate_guess)&&(bitratesearch.rate_guess>0))
    {
        fb=bitratesearch.rate_guess;
        BitrateEstimate(bitratesearch.rate_guess);
    }

    freq_offset_est=bitratesearch.freq_offset_guess;

    //emit the frequency result to whoever wants it
    if(emptyingcountdown<=0)emit FreqOffsetEstimate(freq_offset_est);
     else {emptyingcountdown--;emit FreqOffsetEstimate(0);}

    return;

}

