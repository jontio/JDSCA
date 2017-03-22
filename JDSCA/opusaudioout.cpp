#include "opusaudioout.h"

#include <QDebug>

OpusAudioOut::OpusAudioOut(QObject *parent):
    QObject(parent)
{
    int err;
    decoder = opus_decoder_create(SAMPLE_RATE, CHANNELS, &err);
    if (err<0)qDebug()<<"failed to create decoder: %s\n"<<opus_strerror(err);
    jsound = new TJCSound(this);
    jsound->options.streamName="Opus";
    jsound->iParameters.nChannels=0;
    jsound->oParameters.nChannels=2;
    jsound->audioformat=RTAUDIO_SINT16;
    jsound->sampleRate=48000;
    jsound->bufferFrames=8096;
    jsound->wantedOutDeviceName.clear();
    OpusDCDchanged(false);
    connect(jsound,SIGNAL(SoundEvent(qint16*,qint16*,int)),this,SLOT(SoundEvent(qint16*,qint16*,int)),Qt::DirectConnection);

    //in case OpusDCDchanged() doesn't get heard. sometimes seems to happen when modes are changed
    infotimer = new QTimer(this);
    connect(infotimer,SIGNAL(timeout()),this,SLOT(on_infotimer()));
    infotimer->start(2000);

}

void OpusAudioOut::on_infotimer()
{
    OpusDCDchanged(!opus_buffer_empty);
}

OpusAudioOut::~OpusAudioOut()
{

}

QString OpusAudioOut::get_soundcard_name()
{
    return jsound->wantedOutDeviceName;
}

void OpusAudioOut::set_soundcard_name(QString &name)
{
    if(jsound->wantedOutDeviceName==name)return;
    jsound->Active(false);
    jsound->wantedOutDeviceName=name;

    callback_mutex.lock();
    opus_ratechange=0;
    opus_rate=1.0;
    opus_buffer.fill(0,50*jsound->bufferFrames);
    opus_buf_ptr_head=0;
    opus_buf_ptr_tail=0;
    opus_buffer_use_percentage=0;
    opus_val=0;

    opus_buffer_empty=true;
    opus_buffer_empty_last=true;
    opus_signal=false;

    callback_mutex.unlock();

    OpusDCDchanged(false);

    jsound->Active(true);
}

void OpusAudioOut::decode(QByteArray &packet)
{
    int frame_size = opus_decode(decoder, (uchar*)packet.data(), packet.size(), out, MAX_FRAME_SIZE, 0);
 //   if (frame_size<0)qDebug()<<"decoder failed: "<<opus_strerror(frame_size);
 //    else qDebug()<<"ok frame_size="<<frame_size;

    callback_mutex.lock();

    samples_received+=frame_size;

    for(int i=0;i<frame_size*CHANNELS;i++)
    {
        //static int speedup=0;speedup++;speedup%=100;
        //if(!speedup){samples_received--;continue;}

        opus_buffer[opus_buf_ptr_head]=out[i];
        opus_buf_ptr_head++;opus_buf_ptr_head%=opus_buffer.size();
        if(opus_buf_ptr_head==opus_buf_ptr_tail)
        {
            qDebug()<<"opus input Overflow";
            //this should only happen when the buffer is spooling up
            //set tail to be as far away as posible from the head
            opus_buffer.fill(0);
            opus_buf_ptr_tail=opus_buf_ptr_head+opus_buffer.size()/2;
            opus_buf_ptr_tail%=opus_buffer.size();
        }
    }
    callback_mutex.unlock();

}

void OpusAudioOut::SoundEvent(qint16* inputBuffer,qint16* outputBuffer,int nBufferFrames)
{

    Q_UNUSED(inputBuffer);
    callback_mutex.lock();

    assert(jsound->oParameters.nChannels==2);

    samples_played+=nBufferFrames;

    //calculate opus buffer usage
    opus_buffer_use_percentage=((double)(opus_buf_ptr_head-opus_buf_ptr_tail))/((double)opus_buffer.size());
    if(opus_buffer_use_percentage<0)opus_buffer_use_percentage=1.0+opus_buffer_use_percentage;

    //prevent samples_received and samples_played becoming to big
    if(samples_received>2*48000*60*20||samples_played>2*48000*60*20)
    {
        samples_received/=2;
        samples_played/=2;
    }

    if(opus_buf_ptr_head==opus_buf_ptr_tail)opus_buffer_empty=true;
     else opus_buffer_empty=false;

    if((!opus_buffer_empty)&&opus_buffer_empty_last)
    {
        samples_received=0;
        samples_played=0;
        opus_val=0;
        opus_ratechange=0;
        opus_signal=true;
        current_number_of_samples_to_take=1;
        opus_buf_ptr_tail=opus_buf_ptr_head+opus_buffer.size()/2;
        opus_buf_ptr_tail%=opus_buffer.size();
        OpusDCDchanged(true);
    }

    opus_buffer_empty_last=opus_buffer_empty;

    //estimate sample rate of the opus encoder
    double opus_rate_estimate=((double)samples_played)/((double)samples_received);
    if(samples_played<48000*60*5)opus_rate_estimate=1.0;// dont estimate untill 5 mins has passed

//    static int slo=0;slo++;slo%=10;if(!slo)qDebug()<<samples_received<<samples_played<<" estimate rate="<<opus_rate_estimate<<" buffer use="<<opus_buffer_use_percentage;

    for(int i=0;i<nBufferFrames*2;i+=2)//each frame has two channels
    {

        //check for under run
        if(opus_buf_ptr_head==opus_buf_ptr_tail)
        {
            if(opus_signal)qDebug()<<"-->opus input UnderRun";
            if(opus_signal)OpusDCDchanged(false);
            opus_signal=false;
            opus_buffer.fill(0);
            opus_val=0;
            int k=i;
            for(;k<nBufferFrames*2;k+=2)
            {
                outputBuffer[k]=0;
                outputBuffer[k+1]=0;
            }
            i=k;
            break;
        }

        //keep bounds on the rate estimate
        if(opus_rate_estimate<0.8)opus_rate_estimate=0.9;
        if(opus_rate_estimate>1.2)opus_rate_estimate=1.1;
        if(isnan(opus_rate_estimate))opus_rate_estimate=1.0;

        //update the number of samples we have to take based on our rate estiamte
        current_number_of_samples_to_take+=1.0/opus_rate_estimate;
        if(current_number_of_samples_to_take>4.0)current_number_of_samples_to_take=4.0;
        if(current_number_of_samples_to_take<-1.0)current_number_of_samples_to_take=-1.0;

        //steer sample rate slowly based on buffer useage.
        static bool steeringup=false;
        static bool steeringdown=false;
        if(opus_buffer_use_percentage<0.30){steeringup=true;steeringdown=false;}
        if(opus_buffer_use_percentage>0.70){steeringup=false;steeringdown=true;}
        if(steeringup)
        {
            current_number_of_samples_to_take-=0.001;
            if(opus_buffer_use_percentage>0.5){steeringup=false;steeringdown=false;}
        }
        if(steeringdown)
        {
            current_number_of_samples_to_take+=0.001;//1 samples per 1000
            if(opus_buffer_use_percentage<0.5){steeringup=false;steeringdown=false;}
        }


        //clear as many frames as needed
        while(current_number_of_samples_to_take>=1.0)
        {
            current_number_of_samples_to_take-=1.0;
            //take sample from opus decoder
            opus_val=opus_buffer[opus_buf_ptr_tail];
            opus_buf_ptr_tail++;opus_buf_ptr_tail%=opus_buffer.size();
        }

        outputBuffer[i]=opus_val;
        outputBuffer[i+1]=outputBuffer[i];

        //check for under run
        if(opus_buf_ptr_head==opus_buf_ptr_tail)
        {
           // qDebug()<<"opus input UnderRun";
            if(opus_signal)OpusDCDchanged(false);
            opus_signal=false;
            opus_buffer.fill(0);
            opus_val=0;
            int k=i;
            for(;k<nBufferFrames*2;k+=2)
            {
                outputBuffer[k]=0;
                outputBuffer[k+1]=0;
            }
            i=k;
            break;
        }

    }
    callback_mutex.unlock();

}
