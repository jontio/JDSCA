#include "DSCADataDeFormatter.h"
#include <QtEndian>

//--slip

TSlip::TSlip()
{
    escapeing=false;
    RxPacket.reserve(1024);
    LastGotRXPacketWasTrue=true;
    goodBytes_cnt=0;
    badBytes_cnt=0;
}
void TSlip::NewRxData()
{
    rxi=0;
}
bool TSlip::GotRXPacket(const QByteArray &data)
{

    privateRXdataPtr=&data;

    quint16 crc16_rec;

    if(LastGotRXPacketWasTrue)
    {
        RxPacket.resize(0);
        LastGotRXPacketWasTrue=false;
    }
    while(rxi<privateRXdataPtr->count())
    {


        if(((uchar)(privateRXdataPtr->at(rxi)))==END)
        {
            escapeing=false;
            rxi++;
            if(RxPacket.count()>3)
            {
                crc16.calcusingbytes(RxPacket.data(),RxPacket.size()-3);
                crc16_rec=(((((quint16)(RxPacket.at(RxPacket.count()-2)))<<8)&0xFF00)|(((quint16)(RxPacket.at(RxPacket.count()-1)))&0x00FF));
                if(crc16.crc==crc16_rec)
                {
                    goodBytes_cnt+=RxPacket.size();
                    RxPacket_type=RxPacket.at(RxPacket.count()-3);
                    RxPacket.resize(RxPacket.size()-3);
                    LastGotRXPacketWasTrue=true;
                    //RxPacket.push_back(char(0));
                    return true;
                } else badBytes_cnt+=RxPacket.size();
            }
            RxPacket.resize(0);
            continue;
        }

        if(escapeing)
        {
            switch((uchar)(privateRXdataPtr->at(rxi)))
            {
            case ESC_END:
                    RxPacket.push_back(END);
                    break;
            case ESC_ESC:
                    RxPacket.push_back(ESC);
                    break;
            /*case END:
                    if(RxPacket.count())
                    {
                        rxi++;
                        LastGotRXPacketWasTrue=true;
                        RxPacket.push_back(char(0));
                        escapeing=false;
                        return true;
                    }
                    break;*/
            default:
                    RxPacket.push_back(privateRXdataPtr->at(rxi));
            }
            escapeing=false;
        }
         else
            switch((uchar)(privateRXdataPtr->at(rxi)))
            {
            /*case END:
                    if(RxPacket.count())
                    {
                        rxi++;
                        LastGotRXPacketWasTrue=true;
                        RxPacket.push_back(char(0));
                        return true;
                    }
                    break;*/
            case ESC:
                    escapeing=true;
                    break;
            default:
                    RxPacket.push_back(privateRXdataPtr->at(rxi));
            }

        rxi++;
     }
    return false;
}
QByteArray &TSlip::EscapePacket(uchar packet_type,const QByteArray &data)
{

    uchar hibyte,lowbyte;
    crc16.calcusingbytes(data);
    hibyte=(uchar)((crc16.crc&0xFF00)>>8);
    lowbyte=(uchar)(crc16.crc&0x00FF);

    tmppkt.reserve(data.size()+50);
    tmppkt.resize(0);

 //   tmppkt.push_back(END);//not needed as log as we send END when idling


    for(int i=0;i<data.count();i++)
    {
        switch((uchar)(data.at(i)))
        {
        case END:
                tmppkt.push_back(ESC);
                tmppkt.push_back(ESC_END);
                break;
        case ESC:
                tmppkt.push_back(ESC);
                tmppkt.push_back(ESC_ESC);
                break;
        default:
                tmppkt.push_back((uchar)data.at(i));
        }
    }

    switch(packet_type)
    {
    case END:
            tmppkt.push_back(ESC);
            tmppkt.push_back(ESC_END);
            break;
    case ESC:
            tmppkt.push_back(ESC);
            tmppkt.push_back(ESC_ESC);
            break;
    default:
            tmppkt.push_back(packet_type);
    }

    switch(hibyte)
    {
    case END:
            tmppkt.push_back(ESC);
            tmppkt.push_back(ESC_END);
            break;
    case ESC:
            tmppkt.push_back(ESC);
            tmppkt.push_back(ESC_ESC);
            break;
    default:
            tmppkt.push_back(hibyte);
    }

    switch(lowbyte)
    {
    case END:
            tmppkt.push_back(ESC);
            tmppkt.push_back(ESC_END);
            break;
    case ESC:
            tmppkt.push_back(ESC);
            tmppkt.push_back(ESC_ESC);
            break;
    default:
            tmppkt.push_back(lowbyte);
    }

    tmppkt.push_back(END);

    return tmppkt;
}

//

template<typename T>
DSCADataDeFormatterInterleaver<T>::DSCADataDeFormatterInterleaver()
{
    M=64;

    interleaverowpermute.resize(M);
    interleaverowdepermute.resize(M);

    //this is 1-1 and onto
    for(int i=0;i<M;i++)
    {
        interleaverowpermute[(i*27)%M]=i;
        interleaverowdepermute[i]=(i*27)%M;
//        interleaverowdepermute[(i*19)%M]=i;
    }
    setColumnSize(6);
}
template<typename T>
void DSCADataDeFormatterInterleaver<T>::setColumnSize(int _N)
{
    if(_N<1)return;
    N=_N;
    matrix.resize(M*N);
}
template<typename T>
QVector<T> &DSCADataDeFormatterInterleaver<T>::interleave(QVector<T> &block)
{
    assert(block.size()==(M*N));
    int k=0;
    for(int i=0;i<M;i++)
    {
        for(int j=0;j<N;j++)
        {
            int entry=interleaverowpermute[i]+M*j;
            assert(entry<block.size());
            assert(k<matrix.size());
            matrix[k]=block[entry];
            k++;
        }
    }
    return matrix;
}
template<typename T>
QVector<T> &DSCADataDeFormatterInterleaver<T>::deinterleave(QVector<T> &block)
{
    assert(block.size()==(M*N));
    int k=0;
    for(int j=0;j<N;j++)
    {
        for(int i=0;i<M;i++)
        {
            int entry=interleaverowdepermute[i]*N+j;
            assert(entry<block.size());
            assert(k<matrix.size());
            matrix[k]=block[entry];
            k++;
        }
    }
    return matrix;
}
template<typename T>
QByteArray &DSCADataDeFormatterInterleaver<T>::deinterleave_ba(QVector<T> &block)
{
    matrix_ba.resize(M*N);
    assert(block.size()==(M*N));
    int k=0;
    for(int j=0;j<N;j++)
    {
        for(int i=0;i<M;i++)
        {
            int entry=interleaverowdepermute[i]*N+j;
            assert(entry<block.size());
            assert(k<matrix_ba.size());
            matrix_ba[k]=(char)block[entry];
            k++;
        }
    }
    return matrix_ba;
}
template<typename T>
QVector<T> &DSCADataDeFormatterInterleaver<T>::deinterleave(QVector<T> &block,int cols)
{
    assert(cols<=N);
    assert(block.size()>=(M*cols));
    int k=0;
    for(int j=0;j<cols;j++)
    {
        for(int i=0;i<M;i++)
        {
            int entry=interleaverowdepermute[i]*cols+j;
            assert(entry<block.size());
            assert(k<matrix.size());
            matrix[k]=block[entry];
            k++;
        }
    }
    return matrix;
}
template<typename T>
QSize DSCADataDeFormatterInterleaver<T>::getSize()
{
    QSize size;
    size.setHeight(M);
    size.setWidth(N);
    return size;
}

template class DSCADataDeFormatterInterleaver<int>;
template class DSCADataDeFormatterInterleaver<char>;
template class DSCADataDeFormatterInterleaver<uchar>;
template class DSCADataDeFormatterInterleaver<float>;
template class DSCADataDeFormatterInterleaver<double>;

PreambleDetector::PreambleDetector()
{
    preamble.resize(1);
    buffer.resize(1);
    buffer_ptr=0;
}
void PreambleDetector::setPreamble(QVector<int> _preamble)
{
    preamble=_preamble;
    if(preamble.size()<1)preamble.resize(1);
    buffer.fill(0,preamble.size());
    buffer_ptr=0;
}
bool PreambleDetector::setPreamble(quint64 bitpreamble,int len)
{
    if(len<1||len>64)return false;
    preamble.clear();
    for(int i=len-1;i>=0;i--)
    {
        if((bitpreamble>>i)&1)preamble.push_back(1);
         else preamble.push_back(0);
    }
    if(preamble.size()<1)preamble.resize(1);
    buffer.fill(0,preamble.size());
    buffer_ptr=0;
    return true;
}
bool PreambleDetector::Update(int val)
{ 
    for(int i=0;i<(buffer.size()-1);i++)buffer[i]=buffer[i+1];
    buffer[buffer.size()-1]=val;
    if(buffer==preamble){buffer.fill(0);return true;}
    return false;
}

PreambleDetectorPhaseInvariant::PreambleDetectorPhaseInvariant()
{
    inverted=false;
    preamble.resize(1);
    buffer.resize(1);
    buffer_ptr=0;
    tollerence=0;
}
void PreambleDetectorPhaseInvariant::setPreamble(QVector<int> _preamble)
{
    preamble=_preamble;
    if(preamble.size()<1)preamble.resize(1);
    buffer.fill(0,preamble.size());
    buffer_ptr=0;
}
bool PreambleDetectorPhaseInvariant::setPreamble(quint64 bitpreamble,int len)
{
    if(len<1||len>64)return false;
    preamble.clear();
    for(int i=len-1;i>=0;i--)
    {
        if((bitpreamble>>i)&1)preamble.push_back(1);
         else preamble.push_back(0);
    }
    if(preamble.size()<1)preamble.resize(1);
    buffer.fill(0,preamble.size());
    buffer_ptr=0;
    return true;
}
int PreambleDetectorPhaseInvariant::Update(int val)
{
    assert(buffer.size()==preamble.size());
    int xorsum=0;
    for(int i=0;i<(buffer.size()-1);i++)
    {
        buffer[i]=buffer[i+1];
        xorsum+=buffer[i]^preamble[i];
    }
    xorsum+=val^preamble[buffer.size()-1];
    buffer[buffer.size()-1]=val;
    if(xorsum>=(buffer.size()-tollerence)){buffer.fill(0);inverted=true;return -1;}
    if(xorsum<=tollerence){buffer.fill(0);inverted=false;return 1;}
    return false;
}
void PreambleDetectorPhaseInvariant::setTollerence(int _tollerence)
{
    tollerence=_tollerence;
}


void PuncturedCode::depunture_soft_block(QByteArray &block,int pattern,bool reset)
{
    assert(pattern>=2);
    QByteArray tblock;tblock.reserve(block.size()+block.size()/pattern+1);
    if(reset)depunture_ptr=0;
    for(int i=0;i<block.size();i++)
    {
        depunture_ptr++;
        tblock.push_back(block.at(i));
        if(depunture_ptr>=pattern-1)tblock.push_back(128);
        depunture_ptr%=(pattern-1);
    }

    block=tblock;

}
void PuncturedCode::punture_soft_block(QByteArray &block,int pattern,bool reset)
{
    assert(pattern>=2);
    QByteArray tblock;tblock.reserve(block.size());
    if(reset)punture_ptr=0;
    for(int i=0;i<block.size();i++)
    {
        punture_ptr++;
        if(punture_ptr<(pattern))tblock.push_back(block.at(i));
        punture_ptr%=pattern;
    }
    block=tblock;
}
PuncturedCode::PuncturedCode()
{
    punture_ptr=0;
    depunture_ptr=0;
}

DSCADataDeFormatter::DSCADataDeFormatter(QObject *parent) : QIODevice(parent)
{
    ber=0;

    modecode_counts.fill(0,16);

    FEC_hard_Type=false;

    cntr=1000000000;
    gotsync_last=0;
    blockcnt=-1;

    cntr=1000000000;
    datacdcountdown=0;
    datacd=false;
    emit DataCarrierDetect(datacd);

    QTimer *dcdtimer=new QTimer(this);
    connect(dcdtimer,SIGNAL(timeout()),this,SLOT(updateDCD()));
    dcdtimer->start(1000);

    sbits.reserve(1000);
    decodedbytes.reserve(1000);


    //new viterbi decoder
    jconvolcodec = new JConvolutionalCodec(this);
    QVector<quint16> polys;
    polys.push_back(109);
    polys.push_back(79);
    jconvolcodec->SetCode(2,7,polys);


    dl1.setLength(12);//delay for decode encode BER check
    dl2.setLength(576-6);//delay's data to next frame

    preambledetector.setPreamble(3780831379LL,32);//0x3780831379,0b11100001010110101110100010010011

    //Preamble detector for OQPSK
    preambledetectorphaseinvariantimag.setPreamble(3780831379LL,32);//0x3780831379,0b11100001010110101110100010010011
    preambledetectorphaseinvariantreal.setPreamble(3780831379LL,32);//0x3780831379,0b11100001010110101110100010010011

    mode=Mode::none;
    setSettings(1200,mode1);

}

void DSCADataDeFormatter::setBitRate(double fb)
{
    setSettings(fb,mode);
}

void DSCADataDeFormatter::setMode(DSCADataDeFormatter::Mode _mode)
{
    setSettings(fb,_mode);
}

void DSCADataDeFormatter::setSettings(double _fb,DSCADataDeFormatter::Mode _mode)
{
    if((mode==_mode)&&(fb==_fb))return;
    mode=_mode;
    fb=_fb;
    ifb=qRound(fb);
    preambledetectorphaseinvariantimag.setTollerence(0);
    preambledetectorphaseinvariantreal.setTollerence(0);

    switch(mode)
    {
    default:
        qDebug()<<"DSCADataDeFormatter::setSettings: Invalid mode"<<mode;
    case mode0:
        //no FEC, short frame
        NumberOfBitsInBlock=9600;
        NumberOfBitsInHeader=16;//0 dummy bits
        NumberOfBitsInFrame=NumberOfBitsInHeader+NumberOfBitsInBlock+64;//64 bit preamble
        useingOQPSK=true;
        dl2.setLength(NumberOfBitsInBlock);//delay's data to next frame
        leaver.setColumnSize(NumberOfBitsInBlock/64);//all interleavers have 64 rows
        block.resize(NumberOfBitsInBlock);
        break;
    case mode1:
        //2/3 FEC, short frame
        NumberOfBitsInBlock=9600;
        NumberOfBitsInHeader=16;//0 dummy bits
        NumberOfBitsInFrame=NumberOfBitsInHeader+NumberOfBitsInBlock+64;
        useingOQPSK=true;
        dl2.setLength(3*NumberOfBitsInBlock/4-jconvolcodec->getPaddinglength());//delay's data to next frame
        leaver.setColumnSize(NumberOfBitsInBlock/64);//all interleavers have 64 rows
        block.resize(NumberOfBitsInBlock);
        break;
    case mode2:
        //2/3 FEC, long frame
        NumberOfBitsInBlock=19200;
        NumberOfBitsInHeader=16;//0 dummy bits
        NumberOfBitsInFrame=NumberOfBitsInHeader+NumberOfBitsInBlock+64;
        useingOQPSK=true;
        dl2.setLength(3*NumberOfBitsInBlock/4-jconvolcodec->getPaddinglength());//delay's data to next frame
        leaver.setColumnSize(NumberOfBitsInBlock/64);//all interleavers have 64 rows
        block.resize(NumberOfBitsInBlock);
        break;
    case mode3:
        //1/2 FEC, short frame
        NumberOfBitsInBlock=9600;
        NumberOfBitsInHeader=16;//0 dummy bits
        NumberOfBitsInFrame=NumberOfBitsInHeader+NumberOfBitsInBlock+64;
        useingOQPSK=true;
        dl2.setLength(NumberOfBitsInBlock-jconvolcodec->getPaddinglength());
        leaver.setColumnSize(NumberOfBitsInBlock/64);//all interleavers have 64 rows
        block.resize(NumberOfBitsInBlock);
        break;
    case mode4:
        //1/2 FEC, long frame
        NumberOfBitsInBlock=19200;
        NumberOfBitsInHeader=16;//0 dummy bits
        NumberOfBitsInFrame=NumberOfBitsInHeader+NumberOfBitsInBlock+64;
        useingOQPSK=true;
        dl2.setLength(NumberOfBitsInBlock-jconvolcodec->getPaddinglength());
        leaver.setColumnSize(NumberOfBitsInBlock/64);//all interleavers have 64 rows
        block.resize(NumberOfBitsInBlock);
        break;
    }



    emit signalModeChanged(mode);
}

DSCADataDeFormatter::~DSCADataDeFormatter()
{

}

bool DSCADataDeFormatter::Start()
{
    if(psinkdevice.isNull())return false;
    QIODevice *out=psinkdevice.data();
    out->open(QIODevice::WriteOnly);
    return true;
}

void DSCADataDeFormatter::Stop()
{
    if(!psinkdevice.isNull())
    {
        QIODevice *out=psinkdevice.data();
        out->close();
    }
}

void DSCADataDeFormatter::ConnectSinkDevice(QIODevice *device)
{
    if(!device)return;
    psinkdevice=device;
    Start();
}

void DSCADataDeFormatter::DisconnectSinkDevice()
{
    Stop();
    if(!psinkdevice.isNull())psinkdevice.clear();
}

void DSCADataDeFormatter::updateDCD()
{
    //qDebug()<<datacdcountdown;



    //keep track of the DCD
    if(datacdcountdown>0)datacdcountdown-=3;
     else {if(datacdcountdown<0)datacdcountdown=0;}
    if(datacd&&!datacdcountdown)
    {
        ber=0;
        datacd=false;
        emit DataCarrierDetect(datacd);
    }

    if(!datacd){slip.badBytes_cnt=0;slip.goodBytes_cnt=0;}
    emit signalBER(ber);
}

QByteArray &DSCADataDeFormatter::Decode(const QByteArray &soft_bits)//0 bit --> oldest bit
{
    decodedbytes.clear();


//    //diff deenc test
//    QVector<char> soft_bits2;soft_bits2.resize(soft_bits.size());
//    for(int i=0;i<soft_bits.size();i++)soft_bits2[i]=soft_bits[i];
//    static int lastbit_odd=0;
//    for(int i=0;i<soft_bits2.size();i+=2)
//    {
//        if(((uchar)soft_bits2[i])>128)soft_bits2[i]=1;
//         else soft_bits2[i]=0;

//        int bit=soft_bits2[i]^lastbit_odd;
//        lastbit_odd=soft_bits2[i];

//        if(bit==1)soft_bits2[i]=255;
//         else soft_bits2[i]=0;
//    }
//    static int lastbit_even=0;
//    for(int i=1;i<soft_bits2.size();i+=2)
//    {
//        if(((uchar)soft_bits2[i])>128)soft_bits2[i]=1;
//         else soft_bits2[i]=0;

//        int bit=soft_bits2[i]^lastbit_even;
//        lastbit_even=soft_bits2[i];

//        if(bit==1)soft_bits2[i]=255;
//         else soft_bits2[i]=0;
//    }



    //viterbi delay searching
 //   static int wer2=1;//150*64-100;
 //   static int wer=0;wer++;wer%=5;if(!wer){dl2.setLength(wer2);wer2++;qDebug()<<wer2;}

    quint16 bit=0;
    uchar soft_bit=0;
    for(int i=0;i<soft_bits.size();i++)
    {

        if(((uchar)soft_bits[i])>=128)bit=1;
         else bit=0;

        soft_bit=soft_bits[i];


        //Preamble detector and ambiguity corrector for OQPSK
        int gotsync;
        if(useingOQPSK)
        {
            realimag++;realimag%=2;
            if(realimag)
            {
                gotsync=preambledetectorphaseinvariantimag.Update(bit);
                if(!gotsync_last)
                {
                    gotsync_last=gotsync;
                    gotsync=0;
                } else gotsync_last=0;
            }
            else
            {
                gotsync=preambledetectorphaseinvariantreal.Update(bit);
                if(!gotsync_last)
                {
                    gotsync_last=gotsync;
                    gotsync=0;
                } else gotsync_last=0;
            }


            if(realimag)
            {
                if(preambledetectorphaseinvariantimag.inverted)
                {
                    bit=1-bit;
                    soft_bit=255-soft_bit;
                }
            }
            else
            {
                if(preambledetectorphaseinvariantreal.inverted)
                {
                    bit=1-bit;
                    soft_bit=255-soft_bit;
                }
            }

        }
         else gotsync=preambledetector.Update(bit);


        if(cntr<1000000000)cntr++;
        if(cntr<16)
        {
            if(cntr==0)
            {
                frameinfo=bit;
                infofield.clear();
            }
             else
             {
                frameinfo<<=1;
                frameinfo|=bit;
             }
        }
        if(cntr==15)
        {

            bool vaildframeinfo=false;
            switch(frameinfo)
            {
            case DSCADataDeFormatter::mode_code_0:
                //qDebug()<<"mode0";
                modecode_counts[0]+=3;
                vaildframeinfo=true;
                break;
            case DSCADataDeFormatter::mode_code_1:
                //qDebug()<<"mode1";
                modecode_counts[1]+=3;
                vaildframeinfo=true;
                break;
            case DSCADataDeFormatter::mode_code_2:
                //qDebug()<<"mode2";
                modecode_counts[2]+=3;
                vaildframeinfo=true;
                break;
            case DSCADataDeFormatter::mode_code_3:
                //qDebug()<<"mode3";
                modecode_counts[3]+=3;
                vaildframeinfo=true;
                break;
            case DSCADataDeFormatter::mode_code_4:
                //qDebug()<<"mode4";
                modecode_counts[4]+=3;
                vaildframeinfo=true;
                break;
            default:
            break;
            }
            if(vaildframeinfo&&datacdcountdown<24)datacdcountdown+=2;
            int best_cnt=1;
            int best_mode=DSCADataDeFormatter::none;
            for(int i=0;i<16;i++)
            {
                modecode_counts[i]-=1;
                if(modecode_counts[i]<0)modecode_counts[i]=0;
                if(modecode_counts[i]>4)modecode_counts[i]=4;
                if(modecode_counts[i]>best_cnt)
                {
                    best_cnt=modecode_counts[i];
                    best_mode=i;
                }
            }
            if((best_mode>=0)&&(best_mode!=(int)mode))
            {
                //qDebug()<<"best_mode="<<best_mode;
                setMode((Mode)best_mode);
            }


        }
        if(cntr>=16)
        {

                //fill block
                if(cntr==16)blockcnt=-1;
                int idx=(cntr-NumberOfBitsInHeader)%block.size();
                if(idx<0)idx=0;//for dummy bits drop
                block[idx]=soft_bit;
                if(idx==(block.size()-1))//block is now filled
                {

                    blockcnt++;

                    //deinterleaver
                    QByteArray &deleaveredblock=leaver.deinterleave_ba(block);

                    //FEC decoder
                    QVector<int> deconvol;
                    switch(mode)
                    {
                    case mode0:
                        deconvol=jconvolcodec->Soft_To_Hard_Convert(deleaveredblock);
                        break;
                    case mode1:
                        pc.depunture_soft_block(deleaveredblock,3);
                        if(!FEC_hard_Type)deconvol=jconvolcodec->Decode_Continuous(deleaveredblock);
                         else deconvol=jconvolcodec->Decode_Continuous_hard(deleaveredblock);
                        break;
                    case mode2:
                        pc.depunture_soft_block(deleaveredblock,3);
                        if(!FEC_hard_Type)deconvol=jconvolcodec->Decode_Continuous(deleaveredblock);
                         else deconvol=jconvolcodec->Decode_Continuous_hard(deleaveredblock);
                        break;
                    case mode3:
                        if(!FEC_hard_Type)deconvol=jconvolcodec->Decode_Continuous(deleaveredblock);
                         else deconvol=jconvolcodec->Decode_Continuous_hard(deleaveredblock);
                        break;
                    case mode4:
                        if(!FEC_hard_Type)deconvol=jconvolcodec->Decode_Continuous(deleaveredblock);
                         else deconvol=jconvolcodec->Decode_Continuous_hard(deleaveredblock);
                        break;
                    case none:
                    default:
                        deconvol=jconvolcodec->Soft_To_Hard_Convert(deleaveredblock);
                        break;
                    }


                    //delay line for frame alignment. This is needed for the scrambler
                    dl2.update(deconvol);

                    //scrambler
                    scrambler.reset();
                    scrambler.update(deconvol);

//int ct=0;
//for(int u=0;u<deconvol.size();u++)if(deconvol[u]==1)ct++;
//if(ct<40)for(int u=0;u<deconvol.size();u++)if(deconvol[u]==1)qDebug()<<u<<deconvol.size();

                    //pack the bits into bytes
                    int charptr=0;
                    uchar ch=0;
                    for(int h=0;h<deconvol.size();h++)
                    {
                        ch|=deconvol[h]*128;
                        charptr++;charptr%=8;
                        if(charptr==0)
                        {
                            infofield+=ch;//actual data of information field in bytearray
                            ch=0;
                        }
                        else ch>>=1;
                    }

                    if((cntr-NumberOfBitsInHeader)==(NumberOfBitsInBlock-1))//frame is done when this is true
                    {





                            slip.NewRxData();
                            while(slip.GotRXPacket(infofield))
                            {
                                //slip.RxPacket  these are the packets that were sent. they are check for size but non crc is performed

                                if(slip.RxPacket_type!=1)decodedbytes+="new packet: "+QString::number(slip.RxPacket.size())+" bytes of type "+QString::number(slip.RxPacket_type)+"\n";

                               // decodedbytes.clear();
                                //send packet to anyone interested
                                emit signalDSCAPacket(slip.RxPacket_type,slip.RxPacket);

                                if(datacdcountdown<24)datacdcountdown+=2;

                                switch(slip.RxPacket_type)
                                {
                                case DSCADataDeFormatter::PacketType_RDS:
                                    //decodedbytes+=slip.RxPacket+"\n";
                                    emit signalDSCARDSPacket(slip.RxPacket);
                                    break;
                                case DSCADataDeFormatter::PacketType_OPUS:
                                    //send opus packet to anyone interested
                                    emit signalDSCAOpusPacket(slip.RxPacket);
                                   // decodedbytes+=((QString)"wer2=%1").arg(wer2);
                                   // qDebug()<<(wer2);
                                    break;
                                default:
                                    break;
                                };

                                //keep track of the DCD
                              /*  if(datacdcountdown<12)datacdcountdown+=2;
                                if(!datacd&&datacdcountdown>2)
                                {
                                    datacd=true;
                                    emit DataCarrierDetect(datacd);
                                }*/

                            }



                            int byte_count=(slip.goodBytes_cnt+slip.badBytes_cnt);
                            if(byte_count>20000)
                            {
                                slip.goodBytes_cnt/=2.0;
                                slip.badBytes_cnt/=2.0;
                                if(slip.goodBytes_cnt<0)slip.goodBytes_cnt=0;
                                if(slip.badBytes_cnt<0)slip.badBytes_cnt=0;
                            }
                            ber=((double)(slip.badBytes_cnt))/((double)(byte_count));


//                            double ber_short=((double)(slip.badBytes_cnt))/((double)(byte_count));
//                            qDebug()<<((int)(ber_short*1000.0));



//                            slip.NewRxData();
//                            while(slip.GotRXPacket(infofield))
//                            {
//                                QString decline;
//                                decline+="new packet:"+QString::number(slip.RxPacket.size())+" of type "+QString::number(slip.RxPacket_type)+"\n";
//                                uchar ah=0;
//                                for(int k=0;k<slip.RxPacket.size();k++)
//                                {
//                                    bool isdiff=false;
//                                    uchar ah2=(uchar)slip.RxPacket[k];
//                                    if(ah==ah2)isdiff=false;
//                                     else isdiff=true;
//                                    ah=ah2+1;
//                                    if(isdiff)decline+=((QString)" ++%1").arg(((QString)"").sprintf("%02X", (uchar)slip.RxPacket[k]));
//                                     else decline+=((QString)" __%1").arg(((QString)"").sprintf("%02X", (uchar)slip.RxPacket[k]));
//                                    if(k%20==0)decline+="\n";
//                                }
//                                decline+="\n";
//                                decodedbytes+=decline;
//                                decline.clear();
//                            }


                         //   quint16 crc_calc=crc16.calcusingbytes(&infofieldptr[k*12],12-2);
                         //   quint16 crc_rec=(((uchar)infofield[k*12+12-1])<<8)|((uchar)infofield[k*12+12-2]);






                    }

                }





        }


        if(gotsync)
        {


                if((cntr+1!=NumberOfBitsInFrame)&&datacd)
                {
                    decodedbytes+="Error short frame!!! maybe the soundcard dropped some sound card buffers\n";
                    emit ShortDataFrame();

                    /*datacd=false;
                    datacdcountdown=0;
                    emit DataCarrierDetect(datacd);

                    scrambler.reset();
                    cntr=-1;*/
                }

//else{
//            decodedbytes+=((QString)"Bits for frame = %1\n").arg(cntr+1);
            cntr=-1;
//            decodedbytes+="\nGot sync\n";

            if(!datacd){slip.badBytes_cnt=0;slip.goodBytes_cnt=0;ber=0;}

            //got a signal
            datacd=true;
            datacdcountdown=24;//*4;//depend on frame frate
            emit DataCarrierDetect(datacd);

//     17       scrambler.reset();

 //               }
        }

        if(cntr+1==NumberOfBitsInFrame)
        {
//     17       scrambler.reset();
            cntr=-1;
        }


    }

    if(!datacd)decodedbytes.clear();

    return decodedbytes;
}

qint64 DSCADataDeFormatter::readData(char *data, qint64 maxlen)
{
    Q_UNUSED(data);
    Q_UNUSED(maxlen);
    return 0;
}

qint64 DSCADataDeFormatter::writeData(const char *data, qint64 len)
{
    /*sbits.clear();
    uchar *udata=(uchar*)data;
    uchar auchar;
    for(int i=0;i<len;i++)
    {
        auchar=udata[i];
        for(int j=0;j<8;j++)
        {
            sbits.push_back(auchar&1);
            auchar=auchar>>1;
        }
    }
    Decode(sbits);*/

    QByteArray ba(data,len);
    Decode(ba);

    if(!psinkdevice.isNull())
    {
        QIODevice *out=psinkdevice.data();
        if(out->isOpen())out->write(decodedbytes);
    }

    return len;
}

