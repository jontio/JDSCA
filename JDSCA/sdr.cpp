#include "sdr.h"

using namespace std;


//thread 1
void SDR::rtlsdr_callback(unsigned char *buf, uint32_t len)
{

 //   qDebug()<<"rtlsdr_callback"<<QThread::currentThreadId();
    QMutexLocker locker(&mut);
    quint32 iqpairsreturnedeachcall=len/2;
    if(iqpairsreturnedeachcall<1)return;

    //check if buffers have room for one more
    buffers_mut.lock();
    if(buffers_used>=N_BUFFERS)
    {
        qDebug()<<"Dropped RTL buffer!!";
        buffers_mut.unlock();
        return;
    }
    buffers_mut.unlock();

    //cycle buffers
    buffers_head_ptr%=N_BUFFERS;

    //resize buffer if needed
    buffers[buffers_head_ptr].resize(iqpairsreturnedeachcall/3+3);//1.2M to 400k plus enough for downsampling start of first entry

    //2M
    //convert to complex doubles
    cblock3.resize(iqpairsreturnedeachcall);
    for(quint32 i=0;i<iqpairsreturnedeachcall;++i)
    {
        cblock3[i]=cpx_type(((double)((qint16)buf[2*i] - 127)),((double)((qint16)buf[2*i+1] - 127)));
    }

    //get peak level
    //currently not used
    unsigned char max_in_char=0;
    for(quint32 i=0;i<len;i+=1000)
    {
        if(buf[i]>max_in_char)max_in_char=buf[i];//not quite right but close enough. i just want a peak volume to test for clipping
    }
    peak_input_val=max_in_char-127;

    //11M
    //mix to 0Hz
    mixer->mix(cblock3);

    //about 20M
    //LPF
    switch(filter_selection)
    {

    case FILTER_200_300:
        //LPF 200 to 300. this one should have filtering after the downsampling to 100 if you dont want alising
        for(quint32 i=0;i<iqpairsreturnedeachcall;++i)
        {
            cx_200_300k_BiQuad=cblock3[i]*gain_200_300k_BiQuad_1;cy_200_300k_BiQuad = b0_200_300k_BiQuad_1 * cx_200_300k_BiQuad + cz0_200_300k_BiQuad_1;cz0_200_300k_BiQuad_1 = b1_200_300k_BiQuad_1 * cx_200_300k_BiQuad - a1_200_300k_BiQuad_1 * cy_200_300k_BiQuad + cz1_200_300k_BiQuad_1;cz1_200_300k_BiQuad_1 = b2_200_300k_BiQuad_1 * cx_200_300k_BiQuad - a2_200_300k_BiQuad_1 * cy_200_300k_BiQuad;
            cx_200_300k_BiQuad=cy_200_300k_BiQuad*gain_200_300k_BiQuad_2;cy_200_300k_BiQuad = b0_200_300k_BiQuad_2 * cx_200_300k_BiQuad + cz0_200_300k_BiQuad_2;cz0_200_300k_BiQuad_2 = b1_200_300k_BiQuad_2 * cx_200_300k_BiQuad - a1_200_300k_BiQuad_2 * cy_200_300k_BiQuad + cz1_200_300k_BiQuad_2;cz1_200_300k_BiQuad_2 = b2_200_300k_BiQuad_2 * cx_200_300k_BiQuad - a2_200_300k_BiQuad_2 * cy_200_300k_BiQuad;
            cx_200_300k_BiQuad=cy_200_300k_BiQuad*gain_200_300k_BiQuad_3;cy_200_300k_BiQuad = b0_200_300k_BiQuad_3 * cx_200_300k_BiQuad + cz0_200_300k_BiQuad_3;cz0_200_300k_BiQuad_3 = b1_200_300k_BiQuad_3 * cx_200_300k_BiQuad - a1_200_300k_BiQuad_3 * cy_200_300k_BiQuad + cz1_200_300k_BiQuad_3;cz1_200_300k_BiQuad_3 = b2_200_300k_BiQuad_3 * cx_200_300k_BiQuad - a2_200_300k_BiQuad_3 * cy_200_300k_BiQuad;
            cblock3[i] = cy_200_300k_BiQuad;
        }
        break;

    case FILTER_150_200:
        //LPF 150 to 200
        for(quint32 i=0;i<iqpairsreturnedeachcall;++i)
        {
            cx_150_200k_BiQuad=cblock3[i]*gain_150_200k_BiQuad_1;cy_150_200k_BiQuad = b0_150_200k_BiQuad_1 * cx_150_200k_BiQuad + cz0_150_200k_BiQuad_1;cz0_150_200k_BiQuad_1 = b1_150_200k_BiQuad_1 * cx_150_200k_BiQuad - a1_150_200k_BiQuad_1 * cy_150_200k_BiQuad + cz1_150_200k_BiQuad_1;cz1_150_200k_BiQuad_1 = b2_150_200k_BiQuad_1 * cx_150_200k_BiQuad - a2_150_200k_BiQuad_1 * cy_150_200k_BiQuad;
            cx_150_200k_BiQuad=cy_150_200k_BiQuad*gain_150_200k_BiQuad_2;cy_150_200k_BiQuad = b0_150_200k_BiQuad_2 * cx_150_200k_BiQuad + cz0_150_200k_BiQuad_2;cz0_150_200k_BiQuad_2 = b1_150_200k_BiQuad_2 * cx_150_200k_BiQuad - a1_150_200k_BiQuad_2 * cy_150_200k_BiQuad + cz1_150_200k_BiQuad_2;cz1_150_200k_BiQuad_2 = b2_150_200k_BiQuad_2 * cx_150_200k_BiQuad - a2_150_200k_BiQuad_2 * cy_150_200k_BiQuad;
            cx_150_200k_BiQuad=cy_150_200k_BiQuad*gain_150_200k_BiQuad_3;cy_150_200k_BiQuad = b0_150_200k_BiQuad_3 * cx_150_200k_BiQuad + cz0_150_200k_BiQuad_3;cz0_150_200k_BiQuad_3 = b1_150_200k_BiQuad_3 * cx_150_200k_BiQuad - a1_150_200k_BiQuad_3 * cy_150_200k_BiQuad + cz1_150_200k_BiQuad_3;cz1_150_200k_BiQuad_3 = b2_150_200k_BiQuad_3 * cx_150_200k_BiQuad - a2_150_200k_BiQuad_3 * cy_150_200k_BiQuad;
            cx_150_200k_BiQuad=cy_150_200k_BiQuad*gain_150_200k_BiQuad_4;cy_150_200k_BiQuad = b0_150_200k_BiQuad_4 * cx_150_200k_BiQuad + cz0_150_200k_BiQuad_4;cz0_150_200k_BiQuad_4 = b1_150_200k_BiQuad_4 * cx_150_200k_BiQuad - a1_150_200k_BiQuad_4 * cy_150_200k_BiQuad + cz1_150_200k_BiQuad_4;cz1_150_200k_BiQuad_4 = b2_150_200k_BiQuad_4 * cx_150_200k_BiQuad - a2_150_200k_BiQuad_4 * cy_150_200k_BiQuad;
            cblock3[i] = cy_150_200k_BiQuad;
        }
        break;

    case FILTER_120_150:
        //LPF 120 to 150
        for(quint32 i=0;i<iqpairsreturnedeachcall;++i)
        {
            cx_120_150k_BiQuad=cblock3[i]*gain_120_150k_BiQuad_1;cy_120_150k_BiQuad = b0_120_150k_BiQuad_1 * cx_120_150k_BiQuad + cz0_120_150k_BiQuad_1;cz0_120_150k_BiQuad_1 = b1_120_150k_BiQuad_1 * cx_120_150k_BiQuad - a1_120_150k_BiQuad_1 * cy_120_150k_BiQuad + cz1_120_150k_BiQuad_1;cz1_120_150k_BiQuad_1 = b2_120_150k_BiQuad_1 * cx_120_150k_BiQuad - a2_120_150k_BiQuad_1 * cy_120_150k_BiQuad;
            cx_120_150k_BiQuad=cy_120_150k_BiQuad*gain_120_150k_BiQuad_2;cy_120_150k_BiQuad = b0_120_150k_BiQuad_2 * cx_120_150k_BiQuad + cz0_120_150k_BiQuad_2;cz0_120_150k_BiQuad_2 = b1_120_150k_BiQuad_2 * cx_120_150k_BiQuad - a1_120_150k_BiQuad_2 * cy_120_150k_BiQuad + cz1_120_150k_BiQuad_2;cz1_120_150k_BiQuad_2 = b2_120_150k_BiQuad_2 * cx_120_150k_BiQuad - a2_120_150k_BiQuad_2 * cy_120_150k_BiQuad;
            cx_120_150k_BiQuad=cy_120_150k_BiQuad*gain_120_150k_BiQuad_3;cy_120_150k_BiQuad = b0_120_150k_BiQuad_3 * cx_120_150k_BiQuad + cz0_120_150k_BiQuad_3;cz0_120_150k_BiQuad_3 = b1_120_150k_BiQuad_3 * cx_120_150k_BiQuad - a1_120_150k_BiQuad_3 * cy_120_150k_BiQuad + cz1_120_150k_BiQuad_3;cz1_120_150k_BiQuad_3 = b2_120_150k_BiQuad_3 * cx_120_150k_BiQuad - a2_120_150k_BiQuad_3 * cy_120_150k_BiQuad;
            cx_120_150k_BiQuad=cy_120_150k_BiQuad*gain_120_150k_BiQuad_4;cy_120_150k_BiQuad = b0_120_150k_BiQuad_4 * cx_120_150k_BiQuad + cz0_120_150k_BiQuad_4;cz0_120_150k_BiQuad_4 = b1_120_150k_BiQuad_4 * cx_120_150k_BiQuad - a1_120_150k_BiQuad_4 * cy_120_150k_BiQuad + cz1_120_150k_BiQuad_4;cz1_120_150k_BiQuad_4 = b2_120_150k_BiQuad_4 * cx_120_150k_BiQuad - a2_120_150k_BiQuad_4 * cy_120_150k_BiQuad;
            cblock3[i] = cy_120_150k_BiQuad;
        }
        break;

    case FILTER_100_120:
        //LPF 100 to 120
        for(quint32 i=0;i<iqpairsreturnedeachcall;++i)
        {
            cx_100_120k_BiQuad=cblock3[i]*gain_100_120k_BiQuad_1;cy_100_120k_BiQuad = b0_100_120k_BiQuad_1 * cx_100_120k_BiQuad + cz0_100_120k_BiQuad_1;cz0_100_120k_BiQuad_1 = b1_100_120k_BiQuad_1 * cx_100_120k_BiQuad - a1_100_120k_BiQuad_1 * cy_100_120k_BiQuad + cz1_100_120k_BiQuad_1;cz1_100_120k_BiQuad_1 = b2_100_120k_BiQuad_1 * cx_100_120k_BiQuad - a2_100_120k_BiQuad_1 * cy_100_120k_BiQuad;
            cx_100_120k_BiQuad=cy_100_120k_BiQuad*gain_100_120k_BiQuad_2;cy_100_120k_BiQuad = b0_100_120k_BiQuad_2 * cx_100_120k_BiQuad + cz0_100_120k_BiQuad_2;cz0_100_120k_BiQuad_2 = b1_100_120k_BiQuad_2 * cx_100_120k_BiQuad - a1_100_120k_BiQuad_2 * cy_100_120k_BiQuad + cz1_100_120k_BiQuad_2;cz1_100_120k_BiQuad_2 = b2_100_120k_BiQuad_2 * cx_100_120k_BiQuad - a2_100_120k_BiQuad_2 * cy_100_120k_BiQuad;
            cx_100_120k_BiQuad=cy_100_120k_BiQuad*gain_100_120k_BiQuad_3;cy_100_120k_BiQuad = b0_100_120k_BiQuad_3 * cx_100_120k_BiQuad + cz0_100_120k_BiQuad_3;cz0_100_120k_BiQuad_3 = b1_100_120k_BiQuad_3 * cx_100_120k_BiQuad - a1_100_120k_BiQuad_3 * cy_100_120k_BiQuad + cz1_100_120k_BiQuad_3;cz1_100_120k_BiQuad_3 = b2_100_120k_BiQuad_3 * cx_100_120k_BiQuad - a2_100_120k_BiQuad_3 * cy_100_120k_BiQuad;
            cx_100_120k_BiQuad=cy_100_120k_BiQuad*gain_100_120k_BiQuad_4;cy_100_120k_BiQuad = b0_100_120k_BiQuad_4 * cx_100_120k_BiQuad + cz0_100_120k_BiQuad_4;cz0_100_120k_BiQuad_4 = b1_100_120k_BiQuad_4 * cx_100_120k_BiQuad - a1_100_120k_BiQuad_4 * cy_100_120k_BiQuad + cz1_100_120k_BiQuad_4;cz1_100_120k_BiQuad_4 = b2_100_120k_BiQuad_4 * cx_100_120k_BiQuad - a2_100_120k_BiQuad_4 * cy_100_120k_BiQuad;
            cblock3[i] = cy_100_120k_BiQuad;
        }
        break;

    default:
        ;
    };

    //qint64 t = getticks();

    //50M using DISCRIM_ATAN2_FAST
    //FM demod
    static quint32 down=0;
    unsigned int buffer_size_returned=0;
    double *buffptr=buffers[buffers_head_ptr].data();
    for(;down<iqpairsreturnedeachcall;down+=3)
    {
        cy=cblock3[down];//drop from 1.2M to 400k sampling

        //remove any dc bias
        static cpx_type avept=0;
        avept=avept*(1.0-0.000001)+0.000001*cy;
        cy-=avept;

        double audiosignal=0;

        //4M (this is about 1/5th of the CPU needed of using std::arg)
        //FM demod
        // see http://www.embedded.com/design/configurable-systems/4212086/DSP-Tricks--Frequency-demodulation-algorithms-
        if(discrim_selection==DISCRIM_APPROX)
        {
            static double aveppower=0;
            aveppower=aveppower*(1.0-0.00001)+0.00001*(cy.real()*cy.real()+cy.imag()*cy.imag());
            aveppower=qMax(aveppower,2.0);
            double qpn=cy.real()-zb1;
            double ipn=cy.imag()-zt1;
            double output=((qpn*zt0)-(ipn*zb0));
            zt1=zt0;
            zb1=zb0;
            zt0=cy.imag();
            zb0=cy.real();
            audiosignal=(0.33/aveppower)*output;
        }

        //20M
        //FM demod using angle
        //using angle
        if(discrim_selection==DISCRIM_ATAN2)
        {
            static cpx_type cy_last;
            double angle = -std::arg(cy*std::conj(cy_last));
            cy_last=cy;
            audiosignal=angle/3.0;
        }

        //9M
        //FM demod using another version of atan2
        if(discrim_selection==DISCRIM_ATAN2_FAST)
        {
            static cpx_type cy_last;
            double angle = -arctan2_fast_maybe(cy*std::conj(cy_last));
            cy_last=cy;
            audiosignal=angle/3.0;
        }

        //AGC (DC bias of FM is related to Frequency offset)
        static double avedcbias=0;
        avedcbias=avedcbias*(1.0-0.0001)+0.0001*audiosignal;
        audiosignal-=avedcbias;
        static int down3=0;down3++;
        if(down3>=10)
        {
            down3=0;
            if(useafc)mixer->nudgefreq(5.0*avedcbias,250000-100000,250000+100000);
        }

        //40M
        //wow this uses a lot, it's a FIR thats why
        //SCA down. Frequency shift down and band pass
        cpx_type ctout=hfir->Update_Single_c_out2(audiosignal);
        ctout*=mixer3->take_step_back_and_get_val();
        audiosignal=ctout.real()+ctout.imag();

        //down sample to 200khz (0 to +100k are usable)
        static int down2=0;down2++;
        if(down2<4)continue;
        down2=0;

        //save sample for someone else
        *buffptr=audiosignal;
        buffptr++;
        ++buffer_size_returned;

    }
    down-=iqpairsreturnedeachcall;

    //t = getticks() - t;
    //static qint64 ast=t;
    //ast=ast*0.99+0.01*t;
    //qDebug()<<ast;

    buffers_size_valid[buffers_head_ptr]=buffer_size_returned;//save the actual buffer size to the user


    //advertive we have filled a buffer
    buffers_mut.lock();
    ++buffers_used;
    buffers_not_empty.wakeAll();
    buffers_mut.unlock();

    //goto next buffer
    buffers_head_ptr++;

    return;
}

//thread 2
bool SDR::demod_dispatcher()
{
    qDebug()<<"demod_dispatcher started";
    while(true)
    {

    //    qDebug()<<"demod_dispatcher"<<QThread::currentThreadId();
        buffers_mut.lock();
        if(buffers_used==0)
        {
            buffers_not_empty.wait(&buffers_mut);
        }
        buffers_mut.unlock();

        //check if reason for waking is to cancel
        if(do_demod_dispatcher_cancel)break;

        //cycle beffers
        buffers_tail_ptr%=N_BUFFERS;

        //load buffer ptr and size
        double *buffptr=buffers[buffers_tail_ptr].data();
        int buffer_size=buffers_size_valid[buffers_tail_ptr];

        //pass the data to someone else. use a directconnection here
        emit audio_signal_out(buffptr,buffer_size,1);

        //goto next buffer
        ++buffers_tail_ptr;

        buffers_mut.lock();
        --buffers_used;
        buffers_mut.unlock();

    }

    qDebug()<<"demod_dispatcher finished";

    return do_demod_dispatcher_cancel;

}

//thread 3
SDR::SDR(QObject *parent) : QObject(parent)
{

    //something to drop the FM signal to 0Hz
    mixer = new WaveTableComplex;

    //something to drop the DSCA signal frequency
    hfir = new QJHilbertFilter(this);
    mixer3 = new WaveTableComplex;

    rtldev=NULL;//need so rtl driver doesn't crash
    SubCarrierFrequencyOffset=-1;//need so the first call to setting the offset will not just return

    StopAndCloseRtl();

    //qDebug()<<"SDR"<<QThread::currentThreadId();

}

SDR::~SDR()
{
    //qDebug()<<"~SDR closing";
    //qDebug()<<"~SDR future_demod_dispatcher"<<QThread::currentThreadId()<<future_demod_dispatcher.isFinished()<<future_demod_dispatcher.isStarted()<<future_demod_dispatcher.isRunning()<<future_demod_dispatcher.isPaused();
    //qDebug()<<"~SDR future_rtlsdr_callback"<<QThread::currentThreadId()<<future_rtlsdr_callback.isFinished()<<future_rtlsdr_callback.isStarted()<<future_rtlsdr_callback.isRunning()<<future_rtlsdr_callback.isPaused();

    StopAndCloseRtl();
    delete mixer;
    delete mixer3;
}

QStringList SDR::GetDeviceNames()
{
    //fill Rtl device list
    QStringList devices_names;
    int devcnt=rtlsdr_get_device_count();
    for(int i=0;i<devcnt;i++)devices_names<<QString::fromLocal8Bit(rtlsdr_get_device_name(i));
    return devices_names;
}

bool SDR::StopRtl()
{

    //stop rtlsdr_callback
    int result=1;
    if(!future_rtlsdr_callback.isFinished())
    {
        qDebug()<<"stopping rtlsdr_callback";
        rtlsdr_cancel_async(rtldev);
        future_rtlsdr_callback.waitForFinished();
        result=future_rtlsdr_callback.result();
        if(result)qDebug()<<"Error stopping thread";
        usleep(250000*10);//100ms seems to be the min
        qDebug()<<"rtlsdr_callback stoped";
    }

    //stop demod_dispatcher
    if(!future_demod_dispatcher.isFinished())
    {
        qDebug()<<"stopping demod_dispatcher";
        buffers_mut.lock();
        do_demod_dispatcher_cancel=true;
        buffers_not_empty.wakeAll();
        buffers_mut.unlock();
        future_demod_dispatcher.waitForFinished();
        do_demod_dispatcher_cancel=false;
        usleep(250000);
        qDebug()<<"demod_dispatcher stoped";
    }

    //clear return buffer state
    //qDebug()<<"clearing buffers";
    buffers_head_ptr=0;
    buffers_tail_ptr=0;
    buffers_used=0;
    for(int i=0;i<N_BUFFERS;i++)buffers[i].clear();

    if(result)return false;
    return true;
}

bool SDR::StopAndCloseRtl()
{
    bool result=StopRtl();
    if(rtlsdr_close(rtldev)==0)rtldev=NULL;

    do_demod_dispatcher_cancel=false;
    active=false;
    buffers_head_ptr=0;
    buffers_tail_ptr=0;
    buffers_used=0;
    peak_input_val=0;
    cx=0;cy=0;


    SetSubCarrierFrequencyOffset(20000);

    //FM demod init
    zt0=0;zt1=0;
    zb0=0;zb1=0;

    //mixer for offset tuning
    mixer->setfreq(250000,1200000);

    peak_input_val=0;

    cblock3.clear();

    return result;
}

bool SDR::OpenRtl(int device_index)
{
    //stop and close before opening
    StopAndCloseRtl();

    //open device
    rtlsdr_open(&rtldev,device_index);
    if (NULL == rtldev)
    {
        return false;
    }

    //load gains
    gains.fill(0,rtlsdr_get_tuner_gains(rtldev, NULL));
    rtlsdr_get_tuner_gains(rtldev, gains.data());

    //set samplerate
    if(rtlsdr_set_sample_rate(rtldev,1200000))
    {
        return false;
    }

    //set gain and AGC to default
    SetGain(50);
    SetAGC(true);

    //set frequency to default
    SetFrequency(91450000);

    return true;

}

void SDR::StartRtl()
{

    StopRtl();

    active=true;

    //dispatcher
    future_demod_dispatcher = QtConcurrent::run(this,&SDR::demod_dispatcher);

    //reset buffer and go
    rtlsdr_reset_buffer(rtldev);
    future_rtlsdr_callback = QtConcurrent::run(rtlsdr_read_async, rtldev,(rtlsdr_read_async_cb_t)rtlsdr_callback_dispatcher, this,0,0);//16384*2);

}

void SDR::SetGain(int dB)
{

    //set gain
    rtlsdr_set_tuner_gain_mode(rtldev,1);

    if(gains.size()<1)return;

    double best_diff=fabs(gains[0]-dB);
    double best_idx=0;
    for(int i=1;i<gains.size();i++)
    {
        double again=gains[i];
        again/=10.0;
        if(fabs(again-dB)<best_diff)
        {
            best_diff=fabs(again-dB);
            best_idx=i;
        }
    }

    rtlsdr_set_tuner_gain(rtldev,best_idx);

}

void SDR::SetAGC(bool enable)
{
    rtlsdr_set_agc_mode(rtldev,(int)enable);
}

void SDR::SetAFC(bool enable)
{
    QMutexLocker locker(&mut);
    mixer->setfreq(250000,1200000);
    useafc=enable;
}

void SDR::SetFilterSelection(SDR::Filter_selection _filter_selection)
{
    filter_selection=_filter_selection;
}

SDR::Filter_selection SDR::GetFilterSelection()
{
    return filter_selection;
}

void SDR::SetFrequency(int freq)
{
    mut.lock();
    mixer->setfreq(250000,1200000);
    mut.unlock();
    rtlsdr_set_center_freq(rtldev,freq+250000);
    freq_rtl=rtlsdr_get_center_freq(rtldev)-250000;
}

int SDR::GetFrequency()
{
    QMutexLocker locker(&mut);
    return (rtlsdr_get_center_freq(rtldev)-mixer->wt_freq);
}

void SDR::SetSubCarrierFrequencyOffset(int freq)
{

    if(freq==SubCarrierFrequencyOffset)return;

    if(freq>100000)freq=100000;
    if(freq<0)freq=0;

    double bw=50000;

    SubCarrierFrequencyOffset=freq;
    QMutexLocker locker(&mut);
    hfir->setSize(512*2);
    hfir->setSecondFilterKernel(QJFilterDesign::BandPassHanning(freq+100,bw+freq-100,400000,512*2));
    mixer3->setfreq(freq,400000);
}

int SDR::GetSubCarrierFrequencyOffset()
{
    return SubCarrierFrequencyOffset;
}
