//---------------------------------------------------------------------------


#include "DSP.h"
#include <QDebug>

//---------------------------------------------------------------------------

using namespace std;

TrigLookUp tringlookup;

TrigLookUp::TrigLookUp()
{
    //--load wavetables (trig)
    SinWT.resize(WTSIZE);
    CosWT.resize(WTSIZE);
    CosINV.resize(WTSIZE);
    CISWT.resize(WTSIZE);
    int cosine,i;
    for(i=0;i<WTSIZE;i++){SinWT[i]=(sin(2*M_PI*((double)i)/WTSIZE));}
    for(i=0;i<WTSIZE;i++){CosWT[i]=(sin(M_PI_2+2*M_PI*((double)i)/WTSIZE));}
    for(i=0;i<WTSIZE;i++)
    {
        CISWT[i]=cpx_type(CosWT[i],SinWT[i]);
        cosine=(2.0*((double)i)/(WTSIZE-1.0))-1.0;
        if (cosine>1)cosine=1;
        else if(cosine<-1)cosine=-1;
        CosINV[i]=acos(cosine);
    }
    //--
}

WaveTable::WaveTable()
{
    last_WTptr=0;
    samplerate=48000;
    freq=1000;
    WTstep=(1000.0)*WTSIZE/(48000);//default
    WTptr=0;
    FractionOfSampleItPassesBy=0.0;
}

WaveTable::WaveTable(int _freq,int _samplerate)
{
    last_WTptr=0;
    freq=_freq;
    samplerate=_samplerate;
    if(freq<0)freq=0;
    WTstep=((double)freq)*WTSIZE/((float)samplerate);
    WTptr=0;
    FractionOfSampleItPassesBy=0.0;
}

WaveTable::~WaveTable()
{

}

double  WaveTable::DistancebetweenWT(double WTptr1, double WTptr2)
{
    double dif1=WTptr1-WTptr2;
    double dif2=WTptr2-WTptr1;
    if(dif1<0)dif1+=WTSIZE;
    if(dif2<0)dif2+=WTSIZE;
    double dif=dif1;
    if(dif2<dif1)dif=-dif2;
    dif=dif/WTSIZE;
    return dif;
}

void  WaveTable::WTnextFrame()
{
    JASSERT(WTptr>=0.0);
    if(WTstep<0)WTstep=0;
    last_WTptr=WTptr;
    WTptr+=WTstep;
    while(((int)WTptr)>=WTSIZE) WTptr-=WTSIZE;
}

cpx_type  WaveTable::WTCISValue()
{
   /* tint=(int)WTptr;
    if(tint>=WTSIZE)tint=0;
    if(tint<0)tint=WTSIZE-1;
    return tringlookup.CISWT[tint];*/

    //simple interpolation coule be used if wanted
    double tWTptr=WTptr+((double)WTSIZE)/4.0;
    while(!signbit(tWTptr-(double)WTSIZE))tWTptr-=(double)(WTSIZE);
    if(signbit(tWTptr))tWTptr=0;
    double tintWTptr=(int)tWTptr;
    cpx_type prompt=tringlookup.CISWT[tintWTptr];
    int i=tintWTptr+1;i%=WTSIZE;
    cpx_type late=tringlookup.CISWT[i];
    double rem=tWTptr-((double)tintWTptr);
    return (prompt*(1.0-rem)+late*rem);
}

double   WaveTable::WTSinValue()
{
    tint=(int)WTptr;
    if(tint>=WTSIZE)tint=0;
    if(tint<0)tint=WTSIZE-1;
    return tringlookup.SinWT[tint];
}

double   WaveTable::WTSinValue(double PlusFractOfSample)
{
    double ts=WTptr;
    ts+=PlusFractOfSample*WTstep;
    if(((int)ts)>=WTSIZE) ts-=WTSIZE;
    tint=(int)ts;
    if(tint>=WTSIZE)tint=0;
    if(tint<0)tint=WTSIZE-1;
    return tringlookup.SinWT[tint];
}

double  WaveTable::WTCosValue()
{
    tint=(int)WTptr;
    if(tint>=WTSIZE)tint=0;
    if(tint<0)tint=WTSIZE-1;
    return tringlookup.CosWT[tint];
}

double   WaveTable::WTCosValue(double PlusFractOfSample)
{
    double ts=WTptr;
    ts+=PlusFractOfSample*WTstep;
    if(((int)ts)>=WTSIZE) ts-=WTSIZE;
    tint=(int)ts;
    if(tint>=WTSIZE)tint=0;
    if(tint<0)tint=WTSIZE-1;
    return tringlookup.CosWT[tint];
}

void  WaveTable::WTsetFreq(int _freq,int _samplerate)
{
    samplerate=_samplerate;
    freq=_freq;
    if(freq<0)freq=0;
    WTstep=((double)freq)*WTSIZE/((float)samplerate);
    while(((int)WTptr)>=WTSIZE) WTptr-=WTSIZE;
}

void  WaveTable::SetFreq(double _freq,int _samplerate)
{
    freq=_freq;
    samplerate=_samplerate;
    if(freq<0)freq=0;
    WTstep=(freq)*((double)WTSIZE)/((float)samplerate);
    while(((int)WTptr)>=WTSIZE) WTptr-=WTSIZE;
}

void  WaveTable::SetFreq(double _freq)
{
    freq=_freq;
    if(freq<0)freq=0;
    WTstep=(freq)*((double)WTSIZE)/samplerate;
}

double WaveTable::GetFreqHz()
{
    return freq;
}

void  WaveTable::IncreseFreqHz(double freq_hz)
{
    freq_hz+=freq;
    SetFreq(freq_hz);
}

void  WaveTable::IncresePhaseDeg(double phase_deg)
{
    phase_deg+=(360.0*WTptr/((double)WTSIZE));
    SetPhaseDeg(phase_deg);
}

void  WaveTable::SetPhaseDeg(double phase_deg)
{
    assert(!std::isnan(phase_deg));
    phase_deg=std::fmod(phase_deg,360.0);
    while(phase_deg<0)phase_deg+=360.0;
    WTptr=(phase_deg/360.0)*((double)WTSIZE);
}

void  WaveTable::SetPhaseCycles(double phase_cycles)//untested
{
    phase_cycles=std::fmod(phase_cycles,1.0);
    while(phase_cycles<0)phase_cycles+=1.0;
    assert(phase_cycles>=0);
    assert(phase_cycles<1);
    WTptr=phase_cycles*((double)WTSIZE);
}


double WaveTable::GetPhaseDeg()
{
    return (360.0*WTptr/((double)WTSIZE));
}

double WaveTable::GetFreqTest()
{
    return WTstep;
}

bool  WaveTable::IfPassesPointNextTime()
{
    FractionOfSampleItPassesBy=WTptr+(WTstep-WTSIZE);
    if(FractionOfSampleItPassesBy<0)return false;
    FractionOfSampleItPassesBy/=WTstep;
    return true;
}

bool  WaveTable::IfPassesPointNextTime(double FractionOfWave)
{
    FractionOfSampleItPassesBy=(FractionOfWave*WTSIZE)-WTptr;
    if(FractionOfSampleItPassesBy<0)FractionOfSampleItPassesBy+=WTSIZE;
    if(FractionOfSampleItPassesBy<WTstep)
    {
        FractionOfSampleItPassesBy=(WTstep-FractionOfSampleItPassesBy)/WTstep;
        return true;
    }
    return false;
}

//this is tricky when there are not many samples per symbol eg 19kb/s at 48sps soundcard --> just 2 or three sound card samples per symbol. with jitter we have to be carful not to go to the next symbol
//the toofast test may or may not work to stop slips. line x certinally works though but requires at least 4 time soundcard rate
bool  WaveTable::IfHavePassedPoint(double FractionOfWave)
{
    static bool toofast=false;

    double t_last_WTptr=last_WTptr;
    double t_WTptr=WTptr;
    double pt=(FractionOfWave*WTSIZE);
    t_last_WTptr-=pt;
    t_WTptr-=pt;
    if(t_last_WTptr<0.0)t_last_WTptr+=WTSIZE;
    if(t_WTptr<0.0)t_WTptr+=WTSIZE;
    //if((t_last_WTptr>WTSIZE_3_4)&&(t_WTptr<WTSIZE_1_4))//line x
    if((t_last_WTptr>=WTSIZE/2.0)&&(t_WTptr<WTSIZE/2.0))
    {

        if(toofast)
        {
            //qDebug()<<"toofast!!!";
            toofast=false;
            return false;
        }

        //FractionOfSampleItPassesBy=(WTstep-t_WTptr)/WTstep;
        FractionOfSampleItPassesBy=t_WTptr/WTstep;//WTSIZE;
        toofast=true;
        return true;
    }
    toofast=false;
    return false;
}


bool  WaveTable::IfPassesPointNextTime_frames(double NumberOfFrames)
{
    FractionOfSampleItPassesBy=NumberOfFrames-WTptr;
    if(FractionOfSampleItPassesBy<0)FractionOfSampleItPassesBy+=WTSIZE;
    if(FractionOfSampleItPassesBy<WTstep)
    {
        FractionOfSampleItPassesBy=(WTstep-FractionOfSampleItPassesBy)/WTstep;
        return true;
    }
    return false;
}





void  WaveTable::SetWTptr(double FractionOfWave,double PlusNumberOfFrames)
{
    while(FractionOfWave>=1)FractionOfWave-=1;
    while(FractionOfWave<0)FractionOfWave+=1;
    WTptr=(FractionOfWave*WTSIZE);
    WTptr+=PlusNumberOfFrames*WTstep;
    while(((int)WTptr)>=WTSIZE)WTptr-=WTSIZE;
    while(((int)WTptr)<0)WTptr+=WTSIZE;
}

//---------------------------------------------------------------------------



FIR::FIR(int _NumberOfPoints)
{
        int i;
        points=0;buff=0;

        NumberOfPoints=_NumberOfPoints;
        buffsize=NumberOfPoints+1;
        points=new double[NumberOfPoints];
        for(i=0;i<NumberOfPoints;i++)points[i]=0;
        buff=new double[buffsize];
        for(i=0;i<buffsize;i++)buff[i]=0;
        ptr=0;
        outsum=0;
}

FIR::~FIR()
{
        if(points)delete [] points;
        if(buff)delete [] buff;
}

double  FIR::FIRUpdateAndProcess(double sig)
{
        buff[ptr]=sig;
        ptr++;if(ptr>=buffsize)ptr=0;//ptr%=buffsize;
        int tptr=ptr;
        outsum=0;
        for(int i=0;i<NumberOfPoints;i++)
        {
                outsum+=points[i]*buff[tptr];
                tptr++;if(tptr>=buffsize)tptr=0;//tptr%=buffsize;
        }
        return outsum;
}

void  FIR::FIRUpdate(double sig)
{
        buff[ptr]=sig;
        ptr++;ptr%=buffsize;
}

double  FIR::FIRProcess(double FractionOfSampleOffset)
{
        double nextp=FractionOfSampleOffset;
        double thisp=1-nextp;
        int tptr=ptr;
        int nptr=ptr;
        outsum=0;
        nptr++;nptr%=buffsize;
        for(int i=0;i<NumberOfPoints;i++)
        {
                outsum+=points[i]*(buff[tptr]*thisp+buff[nptr]*nextp);
                tptr=nptr;
                nptr++;nptr%=buffsize;
        }
        return outsum;
}

double  FIR::FIRUpdateAndProcess(double sig, double FractionOfSampleOffset)
{
        buff[ptr]=sig;
        ptr++;ptr%=buffsize;
        double nextp=FractionOfSampleOffset;
        double thisp=1-nextp;
        int tptr=ptr;
        int nptr=ptr;
        outsum=0;
        nptr++;nptr%=buffsize;
        for(int i=0;i<NumberOfPoints;i++)
        {
                outsum+=points[i]*(buff[tptr]*thisp+buff[nptr]*nextp);
                tptr=nptr;
                nptr++;nptr%=buffsize;
        }
        return outsum;
}

void  FIR::FIRSetPoint(int point, double value)
{
    JASSERT(point>=0);
    JASSERT(point<NumberOfPoints);
    if((point<0)||(point>=NumberOfPoints))return;
    points[point]=value;
}

//-----------------
AGC::AGC(double _SecondsToAveOver,double _Fs)
{
    JASSERT(_Fs>1);
    JASSERT(_Fs<1000000);
    AGCMASz=round(_SecondsToAveOver*_Fs);
    JASSERT(AGCMASz>0);
    AGCMASum=0;
    AGCMABuffer=new double[AGCMASz];
    for(int i=0;i<AGCMASz;i++)AGCMABuffer[i]=0;
    AGCMAPtr=0;
    AGCVal=0;
}

double AGC::Update(double sig)
{
    AGCMASum=AGCMASum-AGCMABuffer[AGCMAPtr];
    AGCMASum=AGCMASum+fabs(sig);
    AGCMABuffer[AGCMAPtr]=fabs(sig);
    AGCMAPtr++;AGCMAPtr%=AGCMASz;
    AGCVal=1.414213562/fmax(AGCMASum/((double)AGCMASz),0.000001);
    AGCVal=fmax(AGCVal,0.000001);
    return AGCVal;
}

AGC::~AGC()
{
    if(AGCMASz)delete [] AGCMABuffer;
}

//---------------------

MovingAverage::MovingAverage(int number)
{
    //MASz=round(number);
    MASz=number;
    JASSERT(MASz>0);
    MASum=0;
    MABuffer=new double[MASz];
    for(int i=0;i<MASz;i++)MABuffer[i]=0;
    MAPtr=0;
    Val=0;
}

void MovingAverage::Zero()
{
    for(int i=0;i<MASz;i++)MABuffer[i]=0;
    MAPtr=0;
    Val=0;
    MASum=0;
}

double MovingAverage::Update(double sig)
{
    MASum=MASum-MABuffer[MAPtr];
    MASum=MASum+fabs(sig);
    MABuffer[MAPtr]=fabs(sig);
    MAPtr++;MAPtr%=MASz;
    Val=MASum/((double)MASz);
    return Val;
}

double MovingAverage::UpdateSigned(double sig)
{
    MASum=MASum-MABuffer[MAPtr];
    MASum=MASum+(sig);
    MABuffer[MAPtr]=(sig);
    MAPtr++;MAPtr%=MASz;
    Val=MASum/((double)MASz);
    return Val;
}

MovingAverage::~MovingAverage()
{
    if(MASz)delete [] MABuffer;
}
//---------------------

MSEcalc::MSEcalc(int number)
{
    pointmean = new MovingAverage(number);
    msema = new MovingAverage(number);
    mse=0;
}
MSEcalc::~MSEcalc()
{
    delete pointmean;
    delete msema;
}
void MSEcalc::Zero()
{
    pointmean->Zero();
    msema->Zero();
    mse=0;
}
double MSEcalc::Update(const cpx_type &pt_qpsk)
{
    double tda,tdb;
    cpx_type tcpx;
    pointmean->Update(std::abs(pt_qpsk));
    double mu=pointmean->Val;
    if(mu<0.000001)mu=0.000001;
    tcpx=sqrt(2)*pt_qpsk/mu;
    tda=(fabs(tcpx.real())-1.0);
    tdb=(fabs(tcpx.imag())-1.0);
    mse=msema->Update((tda*tda)+(tdb*tdb));
    return mse;
}

//---------------------
MovingVar::MovingVar(int number)
{
    E = new MovingAverage(number);
    E2 = new MovingAverage(number);
}

double MovingVar::Update(double sig)
{
    E2->Update(sig*sig);
    Mean=E->Update(sig);
    Var=(E2->Val)-(E->Val*E->Val);
    return Var;
}

MovingVar::~MovingVar()
{
    delete E;
    delete E2;
}
//---------------------

MSKEbNoMeasure::MSKEbNoMeasure(int number)
{
    E = new MovingAverage(number);
    E2 = new MovingAverage(number);
}

double MSKEbNoMeasure::Update(double sig)
{
    E2->Update(sig*sig);
    Mean=E->Update(sig);
    Var=(E2->Val)-(E->Val*E->Val);
    double alpha=sqrt(2)/Mean;
    //double tebno=10.0*(log10(2.0)-log10(((Var*alpha*alpha)- 0.0085 ))+log10(1.5));//0.0085 for the non constant modulus after the matched filter
    double tebno=10.0*(log10(2.0)-log10(((Var*alpha*alpha)- 0.0085 )))-5.0;// this one matches matlab better
    if(std::isnan(tebno))tebno=50;
    if(tebno>50.0)tebno=50;
    EbNo=EbNo*0.8+0.2*tebno;
    return EbNo;
}

MSKEbNoMeasure::~MSKEbNoMeasure()
{
    delete E;
    delete E2;
}


//----------------------


DiffDecode::DiffDecode()
{
    laststate=false;
}

bool DiffDecode::Update(bool state)
{
    bool res=(state+laststate)%2;
    laststate=state;
    return res;
}

//-----------------------

bool  BaceConverter::LoadSymbol(int Value)
{
        if(Value<0)
        {
                Value=0;
                ErasurePtr=BuffPosPtr+NumberOfBitsForInSize;
        }

        Buff|=(Value<<BuffPosPtr);
        BuffPosPtr+=NumberOfBitsForInSize;
        if(BuffPosPtr>=NumberOfBitsForOutSize)DataAvailable=true;
         else DataAvailable=false;
        return DataAvailable;
}

bool  BaceConverter::GetNextSymbol()
{
         if(BuffPosPtr>=NumberOfBitsForOutSize)
         {
                if(ErasurePtr>0)
                {
                        Result=-1;
                        ErasurePtr-=NumberOfBitsForOutSize;
                }
                 else Result=Buff&OutMaxVal;
                Buff=(Buff>>NumberOfBitsForOutSize);
                BuffPosPtr-=NumberOfBitsForOutSize;
         }
         if(BuffPosPtr>=NumberOfBitsForOutSize)DataAvailable=true;
          else DataAvailable=false;
         return DataAvailable;
}

 BaceConverter::BaceConverter()
{
        SetInNumberOfBits(4);
        SetOutNumberOfBits(8);
}


void  BaceConverter::SetInNumberOfBits(int NumberOfBits)
{
        InMaxVal=((int)pow(2,NumberOfBits))-1;
        NumberOfBitsForInSize=NumberOfBits;
        Reset();
}

void  BaceConverter::SetOutNumberOfBits(int NumberOfBits)
{
        OutMaxVal=((int)pow(2,NumberOfBits))-1;
        NumberOfBitsForOutSize=NumberOfBits;
        Reset();
}


void  BaceConverter::Reset()
{
        Result=-1;
        DataAvailable=false;
        Buff=0;
        BuffPosPtr=0;
        Result=0;
        ErasurePtr=0;
}

//-----

IIR::IIR()
{
    a.resize(3);
    b.resize(3);
    b[0]=0.00032714218939589035;
    b[1]=0;
    b[2]=0.00032714218939589035;
    a[0]=1;
    a[1]=-0.39005299948210803;
    a[2]= 0.99934571562120822;
    init();
}

void IIR::init()
{
    JASSERT(a.size()>=1);
    JASSERT(b.size()>=1);
    buff_x.resize(b.size());
    buff_y.resize(a.size()-1);
    buff_x_ptr=0;
    buff_y_ptr=0;
    buff_x.fill(0);
    buff_y.fill(0);
}

double  IIR::update(double sig)
{

    buff_x.resize(b.size());
    buff_y.resize(a.size()-1);
    if(buff_x_ptr>=buff_x.size())buff_x_ptr=0;
    if(buff_y_ptr>=buff_y.size())buff_y_ptr=0;

    ASSERTCH(buff_x,buff_x_ptr);
    buff_x[buff_x_ptr]=sig;
    buff_x_ptr++;buff_x_ptr%=buff_x.size();

    y=0;

    //int tp=buff_x_ptr;

    for(int i=b.size()-1;i>=0;i--)
    {
        ASSERTCH(buff_x,buff_x_ptr);
        ASSERTCH(b,i);
        y+=buff_x[buff_x_ptr]*b[i];
        buff_x_ptr++;buff_x_ptr%=buff_x.size();
    }

    for(int i=a.size()-1;i>=1;i--)
    {
        ASSERTCH(buff_y,buff_y_ptr);
        ASSERTCH(a,i);
        y-=buff_y[buff_y_ptr]*a[i];
        buff_y_ptr++;buff_y_ptr%=buff_y.size();
    }

    ASSERTCH(a,0);
    y/=a[0];

    ASSERTCH(buff_y,buff_y_ptr);
    buff_y[buff_y_ptr]=y;
    buff_y_ptr++;buff_y_ptr%=buff_y.size();

    return y;


/*            a(1)*y(n) = b(1)*x(n) + b(2)*x(n-1) + ... + b(nb+1)*x(n-nb)
                - a(2)*y(n-1) - ... - a(na+1)*y(n-na)*/
}

//----------------

//for alpha==1
//not sure if correct for any other Fs other than 48000 and fb 10500
OQPSKEbNoMeasure::OQPSKEbNoMeasure(int number,double _Fs,double _fb)
{
    Fs=_Fs;
    fb=_fb;
    E = new MovingAverage(number);
    E2 = new MovingAverage(number);
}

double OQPSKEbNoMeasure::Update(double sig)
{

    static int slowdown2=0;slowdown2++;slowdown2%=(int)(Fs/22000);
    if(slowdown2)return EbNo;

    E2->Update(sig*sig);
    Mean=E->Update(sig);
    double MeanSquared=Mean*Mean;
    Var=(E2->Val)-(E->Val*E->Val);
    Var-=(0.024709*MeanSquared);//remove non constant of OQPSK after RRC
    double mvr=(((Fs*MeanSquared/(2.0*fb*Var)))*0.13743);//calibrated using matlab


    if(mvr<0.000000001)mvr=0.000000001;
    static double smvr=smvr*0.8+0.2*mvr;
    static int slowdown=0;slowdown++;slowdown%=10000;
    if(slowdown)return EbNo;


    double tebno=10.0*log10(mvr);
    if(std::isnan(tebno))tebno=50;
    if(tebno>50.0)tebno=50;
    if(tebno<0.0)tebno=0;//no real reason but there is no way that less will be usful
    EbNo=tebno;
    //qDebug()<<tebno;
    return EbNo;


//    double tebno=10.0*log10(mvr);
//    if(std::isnan(tebno))tebno=50;
//    if(tebno>50.0)tebno=50;
//    if(tebno<0.0)tebno=0;//no real reason but there is no way that less will be usful
//    EbNo=EbNo*0.8+0.2*tebno;
//    return EbNo;
}

OQPSKEbNoMeasure::~OQPSKEbNoMeasure()
{
    delete E;
    delete E2;
}

//--

//--measure MER and correct for amplitude

MERMeasure::MERMeasure(int number)
{
    E = new MovingAverage(number);
    E2 = new MovingAverage(number);
    Mean=0;
    MER=0;
}

MERMeasure::~MERMeasure()
{
    delete E;
    delete E2;
}

double MERMeasure::Update(cpx_type &symbol)
{
    E->Update(symbol.real());
    E->Update(symbol.imag());
    double Rrec=fabs(symbol.real());
    double Irec=fabs(symbol.imag());
    Rrec-=E->Val;
    Irec-=E->Val;
    Mean=fmax(E->Val,0.1);
    double derror=Irec*Irec+Rrec*Rrec;
    MER=(10.0*log10((2.0*Mean*Mean)/E2->Update(derror)));
    symbol/=Mean;
    return MER;
}


//---


//---fast Hilbert filter
QJHilbertFilter::QJHilbertFilter(QObject *parent)  : QJFastFIRFilter(parent)
{
    setSize(2048);
}

void QJHilbertFilter::setSize(int N)
{
    N=pow(2.0,(ceil(log2(N))));
    assert(N>1);

    kernel.clear();

    kffsamp_t asample;
    for(int i=0;i<N;i++)
    {

        if(i==N/2)
        {
            asample.i=0;
            asample.r=-1;
            kernel.push_back(asample);
            continue;
        }

        if((i%2)==0)
        {
            asample.i=0;
            asample.r=0;
            kernel.push_back(asample);
            continue;
        }

        asample.r=0;
        asample.i=(2.0/((double)N))/(std::tan(M_PI*(((double)i)/((double)N)-0.5)));
        kernel.push_back(asample);

    }
    setKernel(kernel);
}

QVector<kffsamp_t> QJHilbertFilter::getKernel()
{
    return kernel;
}


//---

//---fast FIR

double sinc_normalized(double val)
{
    if (val==0)return 1.0;
    return (sin(M_PI*val)/(M_PI*val));
}

QVector<kffsamp_t> QJFilterDesign::LowPassHanning(double FrequencyCutOff, double SampleRate, int Length)
{
    QVector<kffsamp_t> h;
    if(Length<1)return h;
    if(!(Length%2))Length++;
    int j=1;
    for(int i=(-(Length-1)/2);i<=((Length-1)/2);i++)
    {
        double w=0.5*(1.0-cos(2.0*M_PI*((double)j)/((double)(Length))));
        kffsamp_t x;
        x.r=(w*(2.0*FrequencyCutOff/SampleRate)*sinc_normalized(2.0*FrequencyCutOff*((double)i)/SampleRate));
        x.i=0;
        h.push_back(x);
        j++;
    }

    return h;

/* in matlab this function is
idx = (-(Length-1)/2:(Length-1)/2);
hideal = (2*FrequencyCutOff/SampleRate)*sinc(2*FrequencyCutOff*idx/SampleRate);
h = hanning(Length)' .* hideal;
*/

}

QVector<kffsamp_t> QJFilterDesign::HighPassHanning(double FrequencyCutOff, double SampleRate, int Length)
{
    QVector<kffsamp_t> h;
    if(Length<1)return h;
    if(!(Length%2))Length++;

    QVector<kffsamp_t> h1;
    QVector<kffsamp_t> h2;
    kffsamp_t zero;zero.i=0;zero.r=0;
    kffsamp_t one;one.i=0;one.r=1.0;
    h2.fill(zero,Length);
    h2[(Length-1)/2]=one;

    h1=LowPassHanning(FrequencyCutOff,SampleRate,Length);
    if((h1.size()==Length)&&(h2.size()==Length))
    {
        kffsamp_t val;
        for(int i=0;i<Length;i++)
        {
            val.r=h2[i].r-h1[i].r;
            val.i=h2[i].i-h1[i].i;
            h.push_back(val);
        }
    }

    return h;
}

QVector<kffsamp_t> QJFilterDesign::BandPassHanning(double LowFrequencyCutOff,double HighFrequencyCutOff, double SampleRate, int Length)
{
    QVector<kffsamp_t> h;
    if(Length<1)return h;
    if(!(Length%2))Length++;

    QVector<kffsamp_t> h1;
    QVector<kffsamp_t> h2;

    h2=LowPassHanning(HighFrequencyCutOff,SampleRate,Length);
    h1=LowPassHanning(LowFrequencyCutOff,SampleRate,Length);

    if((h1.size()==Length)&&(h2.size()==Length))
    {
        kffsamp_t val;
        for(int i=0;i<Length;i++)
        {
            val.r=h2[i].r-h1[i].r;
            val.i=h2[i].i-h1[i].i;
            h.push_back(val);
        }
    }

    return h;
}


QJFastFIRFilter::QJFastFIRFilter(QObject *parent) : QObject(parent)
{
    QVector<kffsamp_t> tvect;
    kernelsize=0;
    kffsamp_t asample;
    asample.r=1;
    asample.i=0;
    tvect.push_back(asample);
    nfft=2;
    cfg=kiss_fastfir_alloc(tvect.data(),tvect.size(),&nfft,0,0);
    reset();
}

int QJFastFIRFilter::setKernel(QVector<kffsamp_t> imp_responce)
{
    int _nfft=imp_responce.size()*4;//rule of thumb
    _nfft=pow(2.0,(ceil(log2(_nfft))));
    return setKernel(imp_responce,_nfft);
}

int QJFastFIRFilter::setKernel(QVector<cpx_type> imp_responce)
{
    int _nfft=imp_responce.size()*4;//rule of thumb
    _nfft=pow(2.0,(ceil(log2(_nfft))));
    return setKernel(imp_responce,_nfft);
}

int QJFastFIRFilter::setKernel(QVector<kffsamp_t> imp_responce,int _nfft)
{
    if(!imp_responce.size())return nfft;
    free(cfg);
    _nfft=pow(2.0,(ceil(log2(_nfft))));
    nfft=_nfft;
    cfg=kiss_fastfir_alloc(imp_responce.data(),imp_responce.size(),&nfft,0,0);
    reset();
    kernelsize=imp_responce.size();
    return nfft;
}

int QJFastFIRFilter::setKernel(QVector<cpx_type> imp_responce,int _nfft)
{
    if(!imp_responce.size())return nfft;
    free(cfg);
    _nfft=pow(2.0,(ceil(log2(_nfft))));
    nfft=_nfft;
    QVector<kffsamp_t> timp_responce;
    timp_responce.resize(imp_responce.size());
    for(int i=0;i<imp_responce.size();i++)
    {
        timp_responce[i].r=imp_responce[i].real();
        timp_responce[i].i=imp_responce[i].imag();
    }
    cfg=kiss_fastfir_alloc(timp_responce.data(),timp_responce.size(),&nfft,0,0);
    reset();
    kernelsize=timp_responce.size();
    return nfft;
}

int QJFastFIRFilter::updateKernel(const QVector<kffsamp_t> &imp_responce)
{
    assert(imp_responce.size()>0);
    if(kernelsize!=imp_responce.size())
    {
        return setKernel(imp_responce);
    }

    size_t nfft=cfg->nfft;
    size_t  n_imp_resp=imp_responce.size();
    const kffsamp_t * imp_resp=imp_responce.data();

    assert((((int)nfft) - ((int)n_imp_resp) + 1)>=0);

    /*zero pad in the middle to left-rotate the impulse response
      This puts the scrap samples at the end of the inverse fft'd buffer */
    cfg->tmpbuf[0] = imp_resp[ n_imp_resp - 1 ];
    for (size_t i=0;i<n_imp_resp - 1; ++i) {
        cfg->tmpbuf[ nfft - n_imp_resp + 1 + i ] = imp_resp[ i ];
    }

    FFTFWD(cfg->fftcfg,cfg->tmpbuf,cfg->fir_freq_resp);

    /* TODO: this won't work for fixed point */
    float scale = 1.0 / ((float)cfg->nfft);

    for (size_t i=0; i < cfg->n_freq_bins; ++i ) {
    #ifdef USE_SIMD
        cfg->fir_freq_resp[i].r *= _mm_set1_ps(scale);
        cfg->fir_freq_resp[i].i *= _mm_set1_ps(scale);
    #else
        cfg->fir_freq_resp[i].r *= scale;
        cfg->fir_freq_resp[i].i *= scale;
    #endif
    }


    return nfft;


}

int QJFastFIRFilter::updateKernel(const QVector<cpx_type> &imp_responce)
{
    assert(imp_responce.size()>0);
    if(kernelsize!=imp_responce.size())
    {
        return setKernel(imp_responce);
    }

    size_t nfft=cfg->nfft;
    size_t  n_imp_resp=imp_responce.size();

    assert((((int)nfft) - ((int)n_imp_resp) + 1)>=0);

    /*zero pad in the middle to left-rotate the impulse response
      This puts the scrap samples at the end of the inverse fft'd buffer */
    cfg->tmpbuf[0].r = imp_responce[ n_imp_resp - 1 ].real();
    cfg->tmpbuf[0].i = imp_responce[ n_imp_resp - 1 ].imag();
    for (size_t i=0;i<n_imp_resp - 1; ++i)
    {
        cfg->tmpbuf[ nfft - n_imp_resp + 1 + i ].r = imp_responce[ i ].real();
        cfg->tmpbuf[ nfft - n_imp_resp + 1 + i ].i = imp_responce[ i ].imag();
    }

    FFTFWD(cfg->fftcfg,cfg->tmpbuf,cfg->fir_freq_resp);

    /* TODO: this won't work for fixed point */
    float scale = 1.0 / ((float)cfg->nfft);

    for (size_t i=0; i < cfg->n_freq_bins; ++i ) {
    #ifdef USE_SIMD
        cfg->fir_freq_resp[i].r *= _mm_set1_ps(scale);
        cfg->fir_freq_resp[i].i *= _mm_set1_ps(scale);
    #else
        cfg->fir_freq_resp[i].r *= scale;
        cfg->fir_freq_resp[i].i *= scale;
    #endif
    }


    return nfft;


}

void QJFastFIRFilter::reset()
{
    kffsamp_t asample;
    asample.r=0;
    asample.i=0;
    remainder.fill(asample,nfft*2);
    idx_inbuf=0;
    remainder_ptr=nfft;


    single_input_output_buf_ptr=0;
    single_input_output_buf.fill(asample,nfft);

}

void QJFastFIRFilter::Update(QVector<kffsamp_t> &data)
{
    Update(data.data(), data.size());
}

void QJFastFIRFilter::Update(kffsamp_t *data,int Size)
{

    //ensure enough storage
    if((inbuf.size()-idx_inbuf)<(size_t)Size)
    {
        inbuf.resize(Size+nfft);
        outbuf.resize(Size+nfft);
    }

    //add data to storage
    memcpy ( inbuf.data()+idx_inbuf, data, sizeof(kffsamp_t)*Size );
    size_t nread=Size;

    //fast fir of storage
    size_t nwrite=kiss_fastfir(cfg, inbuf.data(), outbuf.data(),nread,&idx_inbuf);

    int currentwantednum=Size;
    int numfromremainder=std::min(currentwantednum,remainder_ptr);

    //return as much as posible from remainder buffer
    if(numfromremainder>0)
    {
        memcpy ( data, remainder.data(), sizeof(kffsamp_t)*numfromremainder );

        currentwantednum-=numfromremainder;
        data+=numfromremainder;

        if(numfromremainder<remainder_ptr)
        {
            remainder_ptr-=numfromremainder;
            memcpy ( remainder.data(), remainder.data()+numfromremainder, sizeof(kffsamp_t)*remainder_ptr );
        } else remainder_ptr=0;
    }

    //then return stuff from output buffer
    int numfromoutbuf=std::min(currentwantednum,(int)nwrite);
    if(numfromoutbuf>0)
    {
        memcpy ( data, outbuf.data(), sizeof(kffsamp_t)*numfromoutbuf );
        currentwantednum-=numfromoutbuf;
        data+=numfromoutbuf;
    }

    //any left over is added to remainder buffer
    if(((size_t)numfromoutbuf<nwrite)&&(nwrite>0))
    {
        memcpy ( remainder.data()+remainder_ptr, outbuf.data()+numfromoutbuf, sizeof(kffsamp_t)*(nwrite-numfromoutbuf) );
        remainder_ptr+=(nwrite-numfromoutbuf);
    }


    //if currentwantednum>0 then some items were not changed, this should not happen
    //we should anyways have enough to return but if we dont this happens. this should be avoided else a discontinuity of frames occurs. set remainder to zero and set remainder_ptr to nfft before running to avoid this
    if(currentwantednum>0)
    {
        qDebug()<<"Error: user wants "<<currentwantednum<<" more items from fir filter!";
        remainder_ptr+=currentwantednum;
    }

}

QJFastFIRFilter::~QJFastFIRFilter()
{
    free(cfg);
}

kffsamp_t QJFastFIRFilter::Update_Single(kffsamp_t signal)
{
    kffsamp_t out_signal=single_input_output_buf[single_input_output_buf_ptr];
    single_input_output_buf[single_input_output_buf_ptr]=signal;
    single_input_output_buf_ptr++;single_input_output_buf_ptr%=single_input_output_buf.size();
    if(single_input_output_buf_ptr==0)Update(single_input_output_buf.data(),single_input_output_buf.size());
    return out_signal;
}

cpx_type QJFastFIRFilter::Update_Single(cpx_type signal)
{
    cpx_type out_signal=cpx_type(single_input_output_buf[single_input_output_buf_ptr].r,single_input_output_buf[single_input_output_buf_ptr].i);
    single_input_output_buf[single_input_output_buf_ptr].r=signal.real();
    single_input_output_buf[single_input_output_buf_ptr].i=signal.imag();
    single_input_output_buf_ptr++;single_input_output_buf_ptr%=single_input_output_buf.size();
    if(single_input_output_buf_ptr==0)Update(single_input_output_buf.data(),single_input_output_buf.size());
    return out_signal;
}

double QJFastFIRFilter::Update_Single(double signal)
{
    kffsamp_t out_signal=single_input_output_buf[single_input_output_buf_ptr];
    kffsamp_t tsig;tsig.i=0;tsig.r=signal;
    single_input_output_buf[single_input_output_buf_ptr]=tsig;
    single_input_output_buf_ptr++;single_input_output_buf_ptr%=single_input_output_buf.size();
    if(single_input_output_buf_ptr==0)Update(single_input_output_buf.data(),single_input_output_buf.size());
    return out_signal.r;
}

//-----------


#include <QDebug>
void PeakSearch::Search(const QVector<double> &samplebuffer)
{
    if(best_x.size()!=best_y.size())
    {
        qDebug()<<best_x.size()<<best_y.size();
        int sz=std::max(best_x.size(),best_y.size());
        best_x.resize(sz);
        best_y.resize(sz);
        qDebug()<<sz;
    }
    if(best_x.size()<1)
    {
        best_x.resize(1);
        best_y.resize(1);
    }
    best_x.fill(-10000);
    best_y.fill(-10000);
    for(int k=0;k<best_x.size();k++)
    {
        for(int x=0;x<samplebuffer.size();x++)
        {
            if(samplebuffer[x]>best_y[k])
            {
                bool tooclose=false;
                for(int i=0;i<best_x.size();i++)
                {
                    if(i==k)continue;
                    if(abs(best_x[i]-x)<min_dis)
                    {
                        tooclose=true;
                        break;
                    }
                }
                if(!tooclose)
                {
                    best_x[k]=x;
                    best_y[k]=samplebuffer[x];
                }
            }
        }
    }
}


BitRateSearch::BitRateSearch()
{
    freq_offset_guess=0;
    rate_guess=-1;

    //default settings

    validbitrates.push_back(10500);
    validbitrates.push_back(19000);
    validbitrates.push_back(24000);
    validbitrates.push_back(32000);
    validbitrates.push_back(42000);
    validbitrates.push_back(50000);
    validbitrates.push_back(56000);
    validbitrates.push_back(72000);
    validbitrates.push_back(82000);
    validbitrates.push_back(94000);

    setSetting(192000,0.9,4,100,6,3.0,1.0,5000.0);

}

void BitRateSearch::setSetting(double _Fs,double _fft_smoothing,int number_peaks_to_detect,int min_peak_distance,double _max_rate_diff,double _max_ratio,double _min_prominance,double _lockingbw)
{
    lockingbw=_lockingbw;
    min_prominance=_min_prominance;
    max_ratio=_max_ratio;
    Fs=_Fs;
    fft_smoothing=_fft_smoothing;
    max_rate_diff=_max_rate_diff;
    peaksearch.best_x.resize(number_peaks_to_detect);
    peaksearch.best_y.resize(number_peaks_to_detect);
    peaksearch.min_dis=min_peak_distance;

}

double BitRateSearch::update_direct(const QVector<double> &samplebuffer)
{
    //int nfft=samplebuffer.size();//nfft==from_fft.size()==samplebuffer.size()
    peaksearch.Search(samplebuffer);
    return peaktest(samplebuffer,peaksearch.best_x,Fs);
}

double BitRateSearch::update(const QVector<cpx_type> &from_fft)
{
    int nfft=from_fft.size();//nfft==from_fft.size()==samplebuffer.size()
    samplebuffer.resize(nfft);
    for(int i=0;i<nfft;i++)samplebuffer[i]=samplebuffer[i]*fft_smoothing+(1.0-fft_smoothing)*abs(from_fft[i]);
    return update_direct(samplebuffer);
}

double BitRateSearch::update(const QVector<double> &abs_from_fft)
{
    int nfft=abs_from_fft.size();//nfft==from_fft.size()==samplebuffer.size()
    samplebuffer.resize(nfft);
    for(int i=0;i<nfft;i++)samplebuffer[i]=samplebuffer[i]*fft_smoothing+(1.0-fft_smoothing)*abs_from_fft[i];
    return update_direct(samplebuffer);
}

double freqcntr;
void BitRateSearch::quadraticinterpolate(double &y, double &x,double xcenter, double yleft, double ycenter, double yright)
{
    x=0.5*(yleft-yright)/(yleft-2.0*ycenter+yright);
    y=ycenter-0.25*(yleft-yright)*x;
    x+=xcenter;
}

double BitRateSearch::peaktest(const QVector<double> &samplebuffer,const QVector<int> &best_x, double Fs)
//(const QVector<int> &best_x,const QVector<double> &best_y,double Fs,int nfft)
{
    double best_rate=-1;
    double best_diff=1000000000000;
    double hzperbin=Fs/((double)samplebuffer.size());
    QVector<double> candidate_cost;
    QVector<double> candidate_rate;
    QVector<double> candidate_freq_offset;
    //qDebug()<<" ";
    for(int i=0;i<best_x.size();i++)
    {
        if(best_x[i]<0)continue;
        //qDebug()<<best_x[i]<<((double)best_x[i])*hzperbin<<Fs<<((double)samplebuffer.size());
        for(int k=i+1;k<best_x.size();k++)
        {
            if(best_x[k]<0)continue;

            double y1;
            double y2;
            double x1=best_x[i];
            double x2=best_x[k];

            if((x1-1)<0)break;
            if((x1+1)>=samplebuffer.size())break;
            if((x2-1)<0)continue;
            if((x2+1)>=samplebuffer.size())continue;

            //interpolate
            double xcenter1=best_x[i];
            quadraticinterpolate(y1,x1,xcenter1,samplebuffer[xcenter1-1],samplebuffer[xcenter1],samplebuffer[xcenter1+1]);
            double xcenter2=best_x[k];
            quadraticinterpolate(y2,x2,xcenter2,samplebuffer[xcenter2-1],samplebuffer[xcenter2],samplebuffer[xcenter2+1]);

            //calc freq, rate and costs
            double rate=hzperbin*fabs(x2-x1);
            double freq_offset=-hzperbin*(x2+x1-((double)samplebuffer.size()))/4.0;
            double ratio;
            if(y1>y2)ratio=y1/y2;
             else ratio=y2/y1;
            double prominance=(y1+y2)/2.0;

            //test if results are good enough
            if(ratio>max_ratio)continue;
            if(prominance<min_prominance)continue;

            if(fabs(freq_offset)>((lockingbw/2.0)-0.25*(1.0+rate)))continue;//this means the signal fits in the blue band and takes into account bit rate

            //qDebug()<<"Rate = "<<rate<<" freq offset = "<<freq_offset<<" ratio = "<<ratio<<" prominance = "<<prominance;

            //look for best valid rate that matches this mesured rate
            best_rate=-1;
            best_diff=1000000000000;
            for(int i=0;i<validbitrates.size();i++)
            {
                double rate_diff=fabs(rate-validbitrates[i]);
                if(rate_diff<best_diff)
                {
                    best_diff=rate_diff;
                    best_rate=validbitrates[i];
                }
            }

            //if the best one is within given bounds then add to the candidate list along with a cost
            //ratio is cost it means both peaks are about the same height
            if(best_diff<max_rate_diff)
            {
                candidate_rate.push_back(best_rate);
                candidate_freq_offset.push_back(freq_offset);
                candidate_cost.push_back(ratio);
            }

        }
    }

    //freq_offset_guess=0;//??

    //if no candidates    
    if(candidate_rate.size()<1)
    {
        //qDebug()<<"no good guesses";
        return rate_guess;
    }

    //find best candidate
    best_rate=-1;
    double best_freq_offset=0;
    double best_cost=10000000000;
    for(int i=0;i<candidate_rate.size();i++)
    {
        if(candidate_cost[i]<best_cost)
        {
            best_cost=candidate_cost[i];
            best_rate=candidate_rate[i];
            best_freq_offset=candidate_freq_offset[i];
        }
    }

    //give persistance to the returned estimate. also make sure we get two of the same estimates in a row
    if((best_rate>0)&&(last_best_rate==best_rate))
    {
        rate_guess=best_rate;
        freq_offset_guess=best_freq_offset;
      //  qDebug()<<"good guesses "<<rate_guess<<freq_offset_guess;
    } //else qDebug()<<"no good guesses";
    last_best_rate=best_rate;



    return rate_guess;
}

CMA::CMA()
{
    setSettings(55,0.0005,2.0);
}

CMA::~CMA()
{
}


void CMA::reset()
{
    assert(w.size()==x.size());
    assert(w.size()>2);
    w.fill(0);
    w[w.size()/2]=cpx_type(1.0,0);
    //w[w.size()/4+3]=cpx_type(0.3,0);
    x.fill(0);
    stepsize=firststepsize;
}

void CMA::setSettings(int size, double _stepsize, double _beta)
{
    w.resize(size);
    x.resize(size);
    firststepsize=_stepsize;
    finalstepsize=_stepsize;
    stepsizereductionperpoint=0;
    beta=_beta;
    reset();
}

void CMA::setSettings(int size, double _firststepsize,double _finalstepsize,double _stepsizereductionperpoint, double _beta)
{
    w.resize(size);
    x.resize(size);
    firststepsize=_firststepsize;
    finalstepsize=_finalstepsize;
    stepsizereductionperpoint=_stepsizereductionperpoint;
    beta=_beta;
    reset();
}

cpx_type CMA::slowfir(const cpx_type &in_point)
{

    //fir
    cpx_type y=0;
    for(int i=0;i<x.size();i++)
    {
            x[i]=x[i+1];
    }
    x[x.size()-1]=in_point;
    for(int i=0;i<w.size();i++)
    {
        y+=w[i]*x[i];
    }

//    //fir
//    cpx_type y=0;
//    int last_loc=x.size()-1;
//    for(int i=0;i<last_loc;i++)
//    {
//        x[i]=x[i+1];
//        y+=w[i]*x[i];
//    }
//    x[last_loc]=in_point;
//    y+=w[last_loc]*x[last_loc];

    if(!std::isfinite(abs(y)))
    {
        y=0;
        reset();
    }
    /*if((abs(y))>100.0)
    {
        y=0;
        reset();
    }*/
    if(stepsize>finalstepsize)stepsize-=stepsizereductionperpoint;
    return y;
}

void CMA::updateEqualizerForWhoCaresAboutRotation(const cpx_type &y)
{
    assert(w.size()==x.size());
    cpx_type e=y*(pow(abs(y),2)-beta);
    for(int i=0;i<w.size();i++)
    {
        w[i]+=-stepsize*e*std::conj(x[i]);
    }
}

void CMA::updateEqualizerForFixedSquareShape(const cpx_type &y)
{
    assert(w.size()==x.size());
    if(abs(y)>2.0*beta)return;
    cpx_type e=cpx_type(y.real()*(y.real()*y.real()-beta),y.imag()*(y.imag()*y.imag()-beta));
    for(int i=0;i<w.size();i++)
    {
        w[i]+=-stepsize*e*std::conj(x[i]);
        //w[i]=cpx_type(w[i].real()-stepsize*(x[i].real()*e.real()+x[i].imag()*e.imag()),w[i].imag()-stepsize*(x[i].real()*e.imag()-x[i].imag()*e.real()));

    }

}

void CMA::updateEqualizerFor8PointOQPSKShape(const cpx_type &y,int odd)
{
    assert(w.size()==x.size());
    cpx_type e;
    cpx_type ty=y;
    if(odd)ty=y*cpx_type(cos(M_PI_2),sin(M_PI_2));

    if(fabs(ty.imag())<0.9)e=cpx_type(ty.real()*(ty.real()*ty.real()-beta),ty.imag()*(ty.imag()*ty.imag()-0));
     else e=cpx_type(ty.real()*(ty.real()*ty.real()-beta),0*ty.imag()*(ty.imag()*ty.imag()-beta));

    for(int i=0;i<w.size();i++)
    {
        w[i]+=-stepsize*e*std::conj(x[i]);
        //w[i]=cpx_type(w[i].real()-stepsize*(x[i].real()*e.real()+x[i].imag()*e.imag()),w[i].imag()-stepsize*(x[i].real()*e.imag()-x[i].imag()*e.real()));

    }

}


void CMA::updateEqualizerFor8PointOQPSKShapeParallelVersion(const cpx_type &y,int odd)
{
    cpx_type e;
    cpx_type ty=y;
    if(odd)ty=y*cpx_type(cos(M_PI_2),sin(M_PI_2));
    e=cpx_type(ty.real()*(ty.real()*ty.real()-beta),0);

    for(int i=0;i<w.size();i++)
    {
        w[i]+=-stepsize*e*std::conj(x[i]);
        //w[i]=cpx_type(w[i].real()-stepsize*(x[i].real()*e.real()+x[i].imag()*e.imag()),w[i].imag()-stepsize*(x[i].real()*e.imag()-x[i].imag()*e.real()));

    }

}

