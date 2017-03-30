#include "oqpskdemodulator.h"
#include "gui_classes/qspectrumdisplay.h"

#include <QDebug>
#include <QFile>
#include <QTextStream>

//in qt the sound call back is in the main thread so we dont have to worry about muti thread things
//this is different from rtaudio that gets a different thread for the call back
//rt audio seems faster through.
// use qDebug()<<QThread::currentThreadId(); to see the thread id

OqpskDemodulator::OqpskDemodulator(QObject *parent)
:   QObject(parent)
{

    mut = new QMutex(QMutex::Recursive);

    dcd=false;

    cma_enabled=false;

    afc=false;
    scatterpointtype=SPT_constellation;

    freqestimator_working=false;

    mse=100;

    Fs=48000;
    lockingbw=10500;
    freq_center=8000;
    fb=10500;
    signalthreshold=0.5;//lower is less sensitive
    SamplesPerSymbol=2.0*Fs/fb;

    mixer_center.SetFreq(freq_center,Fs);
    mixer2.SetFreq(freq_center,Fs);

    timer.start();

    spectrumnfft=pow(2,SPECTRUM_FFT_POWER);
    spectrumcycbuff.resize(spectrumnfft);
    spectrumcycbuff_ptr=0;

    bbnfft=pow(2,14);
    bbcycbuff.resize(bbnfft);
    bbcycbuff_ptr=0;

    agc = new AGC(4,Fs);

    mermeasure = new MERMeasure(Fs/2);//half a second

    marg= new MovingAverage(800);
    dt.setLength(400);

    phasepointbuff.resize(2);
    phasepointbuff_ptr=0;

    pointbuff.resize(300);
    pointbuff_ptr=0;

    msecalc = new MSEcalc(400);

    coarsefreqestimate = new CoarseFreqEstimate(this);
    coarsefreqestimate->setSettings(14,lockingbw,fb,Fs);
    connect(this, SIGNAL(BBOverlapedBuffer(const QVector<cpx_type>&)),coarsefreqestimate,SLOT(ProcessBasebandData(const QVector<cpx_type>&)));
    connect(coarsefreqestimate, SIGNAL(FreqOffsetEstimate(double)),this,SLOT(FreqOffsetEstimateSlot(double)),Qt::QueuedConnection);
    connect(coarsefreqestimate, SIGNAL(BitrateEstimate(double)),this,SLOT(BitrateEstimate(double)),Qt::QueuedConnection);

    RootRaisedCosine rrc;
    rrc.design(1,55,48000,10500/2);
    fir_re=new FIR(rrc.Points.size());
    fir_im=new FIR(rrc.Points.size());
    for(int i=0;i<rrc.Points.size();i++)
    {
        fir_re->FIRSetPoint(i,rrc.Points.at(i));
        fir_im->FIRSetPoint(i,rrc.Points.at(i));
    }

    //st delays
    double T=48000.0/5250.0;
    delays.setdelay(1);
    delayt41.setdelay(T/4.0);
    delayt42.setdelay(T/4.0);
    delayt8.setdelay(T/8.0);//??? T/4.0 or T/8.0 ??? need to check

    //st 10500hz resonator at 48000fps
    st_iir_resonator.a.resize(3);
    st_iir_resonator.b.resize(3);
    st_iir_resonator.b[0]=0.00032714218939589035;
    st_iir_resonator.b[1]=0;
    st_iir_resonator.b[2]=0.00032714218939589035;
    st_iir_resonator.a[0]=1;
    st_iir_resonator.a[1]=-0.39005299948210803;
    st_iir_resonator.a[2]= 0.99934571562120822;
    st_iir_resonator.init();

    /*Matlab view
    b(1)=0.00032714218939589035;
    b(2)=0;
    b(3)=0.00032714218939589035;
    a(1)=1;
    a(2)=-0.39005299948210803;
    a(3)=  0.99934571562120822;
    fvtool(b,a);
    */

    //st osc
    st_osc.SetFreq(10500,48000);
    st_osc_ref.SetFreq(10500,48000);

    //ct LPF
    ct_iir_loopfilter.a.resize(3);
    ct_iir_loopfilter.b.resize(3);
    ct_iir_loopfilter.b[0]=0.0010275610653672064;//0.0109734835527828;
    ct_iir_loopfilter.b[1]=0.0020551221307344128;//0.0219469671055655;
    ct_iir_loopfilter.b[2]=0.0010275610653672064;//0.0109734835527828;
    ct_iir_loopfilter.a[0]=1;
    ct_iir_loopfilter.a[1]=-1.9207386815577139  ;//-1.72040443455951;
    ct_iir_loopfilter.a[2]=  0.92509247310306331 ;//0.766899247885339;
    ct_iir_loopfilter.init();

    //create a blue bpf for input
    bluebpf=new QJFastFIRFilter(this);
    bluebpf->setKernel(QJFilterDesign::BandPassHanning(59500,77200,Fs,256-1));

    /*Matlab view
    b(1)=0.0010275610653672064;
    b(2)=0.0020551221307344128;
    b(3)=0.0010275610653672064;
    a(1)=1;
    a(2)=-1.9207386815577139  ;
    a(3)=  0.92509247310306331 ;
    fvtool(b,a);
    */

    //BC
    bc.SetInNumberOfBits(1);
    bc.SetOutNumberOfBits(8);

    //rxdata
    RxDataBits.reserve(2000);//bits

    cma.setSettings(20,0.00001 ,1);

    bpfupdatetimer = new QTimer(this);
    bpfupdatetimer->setSingleShot(true);
    connect(bpfupdatetimer,SIGNAL(timeout()),this,SLOT(setBandPassFilter()));

    //this causes the first settings push to emit that Fs and fb have changed to others
    Fs=-1;
    fb=-1;

}

OqpskDemodulator::~OqpskDemodulator()
{

    if(mut->tryLock(5000))mut->unlock();

    delete agc;
    delete fir_re;
    delete fir_im;
    delete msecalc;
    delete marg;
    delete mermeasure;
    delete coarsefreqestimate;

    delete mut;
}

void OqpskDemodulator::setAFC(bool state)
{
    QMutexLocker locker(mut);
    afc=state;
}

void OqpskDemodulator::setScatterPointType(ScatterPointType type)
{
    QMutexLocker locker(mut);
    scatterpointtype=type;
}

void OqpskDemodulator::setSettings(Settings _settings)
{

    QMutexLocker locker(mut);

    settings=_settings;


    if(_settings.Fs!=Fs)emit SampleRateChanged(_settings.Fs);
    Fs=_settings.Fs;
    if(_settings.fb!=fb)emit BitRateChanged(_settings.fb);
    fb=_settings.fb;
    freq_center=_settings.freq_center;
    lockingbw=_settings.lockingbw;
    if(freq_center>((Fs/2.0)-(lockingbw/2.0)))freq_center=((Fs/2.0)-(lockingbw/2.0));
    signalthreshold=_settings.signalthreshold;
    SamplesPerSymbol=2.0*Fs/fb;

    bbnfft=pow(2,_settings.coarsefreqest_fft_power);
    bbcycbuff.resize(bbnfft);
    bbcycbuff_ptr=0;
    coarsefreqestimate->setSettings(_settings.coarsefreqest_fft_power,2.0*lockingbw/2.0,fb,Fs);

    mixer_center.SetFreq(freq_center,Fs);
    mixer2.SetFreq(freq_center,Fs);

    delete agc;
    agc = new AGC(4,Fs);

    pointbuff.resize(300);
    pointbuff_ptr=0;

    emit Plottables(mixer2.GetFreqHz(),mixer_center.GetFreqHz(),lockingbw);

    //not sure why st_timeing_error_systematic!!!

    st_timeing_error_systematic=0.4;
    if(fb==19000.0&&Fs==48000)st_timeing_error_systematic=0.28;//@19k  @ 48ksps
    if(fb==10500.0&&Fs==48000)st_timeing_error_systematic=0.40;//@10.5k @ 48ksps
    if(fb==10500.0&&Fs==96000)st_timeing_error_systematic=0.43;//@10.5k @ 96ksps

    if(fb==42000.0&&Fs==96000)st_timeing_error_systematic=0.27;//@42k  @ 96ksps
    if(fb==32000.0&&Fs==96000)st_timeing_error_systematic=0.33;//@32k  @ 96ksps
    if(fb==24000.0&&Fs==96000)st_timeing_error_systematic=0.38;//@24k  @ 96ksps
    if(fb==19000.0&&Fs==96000)st_timeing_error_systematic=0.38;//@19k  @ 96ksps

    if(fb==56000.0&&Fs==192000)st_timeing_error_systematic=0.34;//@56k  @ 192ksps
    if(fb==50000.0&&Fs==192000)st_timeing_error_systematic=0.39;//@50k  @ 192ksps
    if(fb==42000.0&&Fs==192000)st_timeing_error_systematic=0.40;//@42k  @ 192ksps
    if(fb==32000.0&&Fs==192000)st_timeing_error_systematic=0.39;//@32k  @ 192ksps
    if(fb==24000.0&&Fs==192000)st_timeing_error_systematic=0.44;//@24k  @ 192ksps
    if(fb==19000.0&&Fs==192000)st_timeing_error_systematic=0.46;//@19k  @ 192ksps
    if(fb==10500.0&&Fs==192000)st_timeing_error_systematic=0.47;//@42k  @ 192ksps


    st_osc.SetFreq(fb,Fs);
    st_osc_ref.SetFreq(fb,Fs);

    RootRaisedCosine rrc;
    int rrclen=(int)(6.0*(Fs/(fb/2.0))+1.0);if(rrclen%2==0)rrclen++;//make the rrc filters about 6 symbols
    rrc.design(1.0,rrclen,Fs,fb/2.0);
    delete fir_re;
    delete fir_im;
    fir_re=new FIR(rrc.Points.size());
    fir_im=new FIR(rrc.Points.size());
    for(int i=0;i<rrc.Points.size();i++)
    {
        fir_re->FIRSetPoint(i,rrc.Points.at(i));
        fir_im->FIRSetPoint(i,rrc.Points.at(i));
    }

    cma.setSettings(24,0.000001 ,1);//24--> 12 symbols , 0.00002 is somewhat slow to converge

    //st delays
    double T=Fs/(fb/2.0);
    delays.setdelay(1);
    delayt41.setdelay(T/4.0);
    delayt42.setdelay(T/4.0);
    delayt8.setdelay(T/8.0);

    //matlab
    //fc=10500;
    //Ts=1/48000;
    //r=0.999672804;

    //b(1)=(1-r^2)/2;
    //b(2)=0;
    //b(3)=-b(1);
    //a(1)=1;
    //a(2)=-2*r*cos(2*pi*fc*Ts);
    //a(3)=r^2;

    //bandwidth
    double r=0.999672804;

    //st. fb hz resonator at Fs fps
    st_iir_resonator.a.resize(3);
    st_iir_resonator.b.resize(3);
    st_iir_resonator.b[0]=(1.0-r*r)/2.0;
    st_iir_resonator.b[1]=0.0;
    st_iir_resonator.b[2]=-st_iir_resonator.b[0];
    st_iir_resonator.a[0]=1.0;
    st_iir_resonator.a[1]=-2.0*r*cos(2.0*M_PI*fb/Fs);
    st_iir_resonator.a[2]=r*r;
    st_iir_resonator.init();

    //ct LPF
    ct_iir_loopfilter.a.resize(3);
    ct_iir_loopfilter.b.resize(3);
    ct_iir_loopfilter.b[0]=0.0010275610653672064;//0.0109734835527828;
    ct_iir_loopfilter.b[1]=0.0020551221307344128;//0.0219469671055655;
    ct_iir_loopfilter.b[2]=0.0010275610653672064;//0.0109734835527828;
    ct_iir_loopfilter.a[0]=1;
    ct_iir_loopfilter.a[1]=-1.9207386815577139  ;//-1.72040443455951;
    ct_iir_loopfilter.a[2]=  0.92509247310306331 ;//0.766899247885339;
    ct_iir_loopfilter.init();

    DCDstatSlot(false);

    mse=1;

    setBandPassFilter();

}

void OqpskDemodulator::CenterFreqChangedSlot(double freq_center)//spectrum display calls this when user changes the center freq
{

    QMutexLocker locker(mut);

    mixer_center.SetFreq(freq_center,Fs);
    if(afc)mixer2.SetFreq(mixer_center.GetFreqHz());
    if((mixer2.GetFreqHz()-mixer_center.GetFreqHz())>(lockingbw/2.0))
    {
        mixer2.SetFreq(mixer_center.GetFreqHz()+(lockingbw/2.0));
    }
    if((mixer2.GetFreqHz()-mixer_center.GetFreqHz())<(-lockingbw/2.0))
    {
        mixer2.SetFreq(mixer_center.GetFreqHz()-(lockingbw/2.0));
    }
    for(int j=0;j<bbcycbuff.size();j++)bbcycbuff[j]=0;
    emit Plottables(mixer2.GetFreqHz(),mixer_center.GetFreqHz(),lockingbw);

    bpfupdatetimer->start(100);
}

void OqpskDemodulator::setBandwidth(double bandwidth_hz)
{

    QMutexLocker locker(mut);

    if(bandwidth_hz<1000.0)return;
    if(bandwidth_hz>Fs/2.0)return;

    //qDebug()<<"bandwidth_hz"<<bandwidth_hz;
    lockingbw=bandwidth_hz;

    if((mixer_center.GetFreqHz()-lockingbw/2.0)<0)
    {
        //lockingbw=2.0*mixer_center.GetFreqHz();
        CenterFreqChangedSlot(lockingbw/2.0);
    }
    if((mixer_center.GetFreqHz()+lockingbw/2.0)>Fs/2.0)
    {
        //lockingbw=2.0*(Fs/2.0-mixer_center.GetFreqHz());
        CenterFreqChangedSlot(Fs/2.0-lockingbw/2.0);
    }
    CenterFreqChangedSlot(mixer_center.GetFreqHz());
}

double  OqpskDemodulator::getCurrentFreq()
{
    QMutexLocker locker(mut);
    return mixer_center.GetFreqHz();
}

void OqpskDemodulator::demodData(const double *inputBuffer, qint64 nBufferFrames,int channels)
{
    double lastmse=mse;

    //qDebug()<<"OqpskDemodulator::demodData"<<QThread::currentThreadId();

    bool sendscatterpoints=false;

    if(channels<0||channels>2)return;//what do we do?

    QMutexLocker locker(mut);

    //const short *ptr = reinterpret_cast<const short *>(inputBuffer);
    //for(int i=0;i<nBufferFrames/((int)sizeof(short));i++)
    double dval;
    for(int i=0;i<nBufferFrames;i++)
    {

        if(channels==2)
        {
            dval=inputBuffer[i*channels]+inputBuffer[i*channels+1];
        }
         else dval=inputBuffer[i];


        //for looks
        static double maxval=0;
        if(fabs(dval)>maxval)maxval=fabs(dval);
        spectrumcycbuff[spectrumcycbuff_ptr]=dval;
        spectrumcycbuff_ptr++;spectrumcycbuff_ptr%=spectrumnfft;
        if(timer.elapsed()>150)
        {
            sendscatterpoints=true;
            timer.start();
            emit OrgOverlapedBuffer(spectrumcycbuff);
            emit PeakVolume(maxval);
            maxval=0;
            //qDebug()<<cma.stepsize*10000.0;
        }


        //bpf filter around our blue range
        //this means we can avoid having to set excess to the correct value and also mean the freq and symbol rate time algos can be made to work even if there is iterference nearby
        dval=bluebpf->Update_Single(dval);

        //for coarse freq estimation
        //if((mse>signalthreshold)&&(!dcd))
        if(!dcd)
        {
            if(!freqestimator_working)
            {
                bbcycbuff[bbcycbuff_ptr]=mixer_center.WTCISValue()*dval;
                bbcycbuff_ptr++;bbcycbuff_ptr%=bbnfft;
                if(bbcycbuff_ptr==0)
                {
                    freqestimator_working=true;
                    emit BBOverlapedBuffer(bbcycbuff);
                }
            }
        }
         else
         {
            bbcycbuff_ptr++;bbcycbuff_ptr%=bbnfft;
            if(bbcycbuff_ptr==0)FreqOffsetEstimateSlot(0);
         }


        //-----


        //mix
        cpx_type cval= mixer2.WTCISValue()*dval;

        //rrc
        cpx_type sig2=cpx_type(fir_re->FIRUpdateAndProcess(cval.real()),fir_im->FIRUpdateAndProcess(cval.imag()));

        //AGC
        sig2*=agc->Update(std::abs(sig2));

        //clipping
        double abval=std::abs(sig2);
        if(abval>2.84)sig2=(2.84/abval)*sig2;

        //will remove some external interference but only once the signal has been locked
        //uses more cpu here for low bit rates so probably still not to bad if we run it
        //in the half symbol period so its just run twice per symbol
        ///if(cma_enabled)sig2=cma.slowfir(sig2);

        //symbol timer error calculation
        double st_diff=delays.update(abval*abval)-(abval*abval);
        double st_d1out=delayt41.update(st_diff);
        double st_d2out=delayt42.update(st_d1out);
        double st_eta=(st_d2out-st_diff)*st_d1out;
        st_eta=st_iir_resonator.update(st_eta);
        cpx_type st_m1=cpx_type(st_eta,-delayt8.update(st_eta));
        cpx_type st_out=st_osc.WTCISValue()*st_m1;
        double st_angle_error=std::arg(st_out);

        //adjust frequency and phase of symbol timing
        double weighting=fabs(tanh(st_angle_error));
        static int stcd=0;if(!dcd)stcd=Fs*3;if(stcd>0)stcd--;//3 seconds of aggression
        if(stcd)
        {
            st_osc.IncreseFreqHz(-weighting*st_angle_error*0.00001*((fb/10500.0)*48000.0/Fs));
            st_osc.AdvanceFractionOfWave(-(1.0-weighting)*st_angle_error*(0.01/360.0)*((fb/10500.0)*48000.0/Fs));
        }
        else
        {
            st_osc.IncreseFreqHz(-weighting*st_angle_error*0.000001*((fb/10500.0)*48000.0/Fs));
            st_osc.AdvanceFractionOfWave(-(1.0-weighting)*st_angle_error*(0.005/360.0)*((fb/10500.0)*48000.0/Fs));
        }

        //set limits on how much the st can change in frequency
        if(st_osc.GetFreqHz()>(st_osc_ref.GetFreqHz()+5.0))st_osc.SetFreq(st_osc_ref.GetFreqHz()+5.0);
        if(st_osc.GetFreqHz()<(st_osc_ref.GetFreqHz()-5.0))st_osc.SetFreq(st_osc_ref.GetFreqHz()-5.0);

        //sample times
        static cpx_type sig2_last=sig2;
        if(st_osc.IfHavePassedPoint(st_timeing_error_systematic))
        {

            //interpol
            double pt_last=st_osc.FractionOfSampleItPassesBy;
            double pt_this=1.0-pt_last;
            cpx_type pt=pt_this*sig2+pt_last*sig2_last;

            static int yui=0;

            //CMA algo
            //will remove some self interference and out of signal band interference. id rather run it for every sample but that uses to much cpu
            if(cma_enabled)
            {
                //the the sample through the filter
                pt=cma.slowfir(pt);

                //cma update. always goes here
                cma.updateEqualizerFor8PointOQPSKShapeParallelVersion(pt,yui);

                //reduce sma stepsize faster once we get a lock
                if(dcd&&cma.stepsize>cma.finalstepsize)cma.stepsize*=0.999999;
            }

            yui++;yui%=2;
            static cpx_type pt_d=0;
            if(!yui)pt_d=pt;
             else
            {
                //create 4 point
                cpx_type pt_qpsk=cpx_type(pt.real(),pt_d.imag());

                //calculate carrier timing error
                double ct_xt=(pt.imag())*pt.real();
                double ct_xt_d=(pt_d.real())*pt_d.imag();
                double ct_ec=ct_xt_d-ct_xt;
                if(ct_ec>M_PI)ct_ec=M_PI;
                if(ct_ec<-M_PI)ct_ec=-M_PI;
                ct_ec=ct_iir_loopfilter.update(ct_ec);

                //adjust carrier frequecy and phase
                double carrier_aggression=1.0;//1.0;//0 to 1. big numbers lock faster but noise is a problem. smaller numbers are a problem if the trasmitter has a large frequency drift
                if(dcd)carrier_aggression=0.5;
                mixer2.IncresePhaseDeg(carrier_aggression*1.0*ct_ec);//*cos(ct_ec));
                mixer2.IncreseFreqHz(carrier_aggression*0.01*ct_ec);//*(1.0-cos(ct_ec)));

                //rotate to remove any remaining rotaional bias
                marg->UpdateSigned(ct_ec/2.0);
                dt.update(pt_qpsk);
                pt_qpsk*=cpx_type(cos(marg->Val),sin(marg->Val));

                //measure MER (seem close to EbNo) and correct any remaining amplitude bias (assumes both freq and symbol timeing for measurment, amp adjust doesnt need anyhting)
                mermeasure->Update(pt_qpsk);

                //calc MSE of the points (frequency invariant, needs symbol timing)
                mse=msecalc->Update(pt_qpsk);

                //gui feedback
                static int slowdown=0;
                if(pointbuff_ptr==0){slowdown++;slowdown%=100*qMax((int)(fb/10500.0),1);}
                //for looks show constellation
                if(!slowdown)
                {
                    ASSERTCH(pointbuff,pointbuff_ptr);
                    if(scatterpointtype==SPT_8constellation)
                    {
                        pointbuff[pointbuff_ptr]=pt;
                        pointbuff_ptr++;pointbuff_ptr%=pointbuff.size();
                        pointbuff[pointbuff_ptr]=pt_d;
                        pointbuff_ptr++;pointbuff_ptr%=pointbuff.size();
                    }
                     else
                     {
                        pointbuff[pointbuff_ptr]=pt_qpsk;
                        pointbuff_ptr++;pointbuff_ptr%=pointbuff.size();
                     }
                    if((scatterpointtype==SPT_constellation||scatterpointtype==SPT_8constellation)&&sendscatterpoints){sendscatterpoints=false;emit ScatterPoints(pointbuff);}
                }
                //for looks show symbol phase offset
                if(!slowdown)
                {
                    cpx_type st_phase_offset_pt=st_osc_ref.WTCISValue()*std::conj(st_osc.WTCISValue());
                    ASSERTCH(phasepointbuff,phasepointbuff_ptr);
                    phasepointbuff[phasepointbuff_ptr]=st_phase_offset_pt;
                    phasepointbuff_ptr++;phasepointbuff_ptr%=phasepointbuff.size();
                    if(scatterpointtype==SPT_phaseoffsetest&&sendscatterpoints){sendscatterpoints=false;emit ScatterPoints(phasepointbuff);}
                }

                //soft BPSK x2
                //-1 --> 0 , 1 --> 255 (-1 means 0 and 1 means 1) sort of
                //there is no packed bits in each byte
                //convert to the strange range for soft
                int ibit=qRound(pt_qpsk.imag()*127.0+128.0);
                if(ibit>255)ibit=255;
                if(ibit<0)ibit=0;
                RxDataBits.push_back((uchar)ibit);
                ibit=qRound(pt_qpsk.real()*127.0+128.0);
                if(ibit>255)ibit=255;
                if(ibit<0)ibit=0;
                RxDataBits.push_back((uchar)ibit);

            }


        }
        sig2_last=sig2;


        //-----

        mixer2.WTnextFrame();
        mixer_center.WTnextFrame();
        st_osc.WTnextFrame();
        st_osc_ref.WTnextFrame();
    }

    //return the demodulated data
    //using bits
    if(!RxDataBits.isEmpty()&&RxDataBits.size()>(fb/20))
    {
        if(mse<signalthreshold||lastmse<signalthreshold)
        {
            QByteArray tba=RxDataBits;
            emit demodulatedSoftBits(tba);
        }
        RxDataBits.clear();
    }



}

void OqpskDemodulator::BitrateEstimate(double bitrate_est)//coarse est class calls this with current est
{
    QMutexLocker locker(mut);
    //qDebug()<<"change rate!!! to"<<bitrate_est;
    settings.fb=bitrate_est;
    settings.freq_center=mixer_center.GetFreqHz();
    settings.lockingbw=lockingbw;
    setSettings(settings);
}

void OqpskDemodulator::FreqOffsetEstimateSlot(double freq_offset_est)//coarse est class calls this with current est
{



    QMutexLocker locker(mut);

    freqestimator_working=false;

    if((mixer_center.GetFreqHz()+freq_offset_est)>Fs/2)freq_offset_est=0;
    if((mixer_center.GetFreqHz()+freq_offset_est)<0)freq_offset_est=0;

    static int countdown=4;
    if(!dcd)
    {
        mixer2.SetFreq(mixer_center.GetFreqHz()+freq_offset_est);
        //st_osc.SetFreq(st_osc_ref.GetFreqHz());//reset st frequency

     //anti lockup test
     st_osc.SetFreq(st_osc_ref.GetFreqHz());

        //reset cma
        cma.setSettings(24,0.0001,0.000001,0.0000000001,1);//start the stepsize very big then reduce it as time goes by
        //cma.setSettings(24,0.000001 ,1);// very slow a but doesn't get put off by noise
        //cma.setSettings(24,0.00002 ,1);//24--> 12 symbols , 0.00002 is somewhat slow to converge

    }    
    if((afc)&&(dcd)&&(fabs(mixer2.GetFreqHz()-mixer_center.GetFreqHz())>3.0))//got a sig, afc on, freq is getting a little to far out
    {
        if(countdown>0)countdown--;
         else
         {
            mixer_center.SetFreq(mixer2.GetFreqHz());
            if(mixer_center.GetFreqHz()<lockingbw/2.0)mixer_center.SetFreq(lockingbw/2.0);
            if(mixer_center.GetFreqHz()>(Fs/2.0-lockingbw/2.0))mixer_center.SetFreq(Fs/2.0-lockingbw/2.0);
            coarsefreqestimate->bigchange();
            for(int j=0;j<bbcycbuff.size();j++)bbcycbuff[j]=0;
            bbcycbuff_ptr=0;
            setBandPassFilter();
         }
    } else countdown=4;
    emit Plottables(mixer2.GetFreqHz(),mixer_center.GetFreqHz(),lockingbw);

    emit EbNoMeasurmentSignal(mermeasure->MER);
    emit MSESignal(mse);
    if(mse>signalthreshold)emit SignalStatus(false);
     else emit SignalStatus(true);

}

void OqpskDemodulator::DCDstatSlot(bool _dcd)
{
    QMutexLocker locker(mut);

    dcd=_dcd;
    if(dcd)signalthreshold=0.95;//if the deformater says we have data comeing from us then dont loose signal till the datadeformatter tells us
     else signalthreshold=0.6;
}

void OqpskDemodulator::setBandPassFilter()
{

    QMutexLocker locker(mut);

    coarsefreqestimate->setSettings(settings.coarsefreqest_fft_power,2.0*lockingbw/2.0,fb,Fs);
    bluebpf->updateKernel(QJFilterDesign::BandPassHanning(mixer_center.GetFreqHz()-lockingbw/2.0,mixer_center.GetFreqHz()+lockingbw/2.0,Fs,bluebpf->getKernelSize()));
}
