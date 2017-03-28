#ifndef DSCADataDeFormatter_H
#define DSCADataDeFormatter_H

#include <QObject>
#include <QPointer>
#include <QVector>
#include <QTimer>
#include <QDateTime>
#include <QList>
#include <QDebug>
#include <assert.h>
#include <QSize>
#include <QThread>

#include "jconvolutionalcodec.h"

class PuncturedCode
{
public:
    PuncturedCode();
    void depunture_soft_block(QByteArray &block,int pattern, bool reset=true);
    void punture_soft_block(QByteArray &block, int pattern, bool reset=true);
private:
    int punture_ptr;
    int depunture_ptr;
};

class CRC16 //this seems to be called GENIBUS not CCITT
{
public:
    quint16 crc;
    CRC16(){}
    quint16 calcusingbits(int *bits,int numberofbits)
    {
        crc = 0xFFFF;
        int crc_bit;
        for(int i=0; i<numberofbits; i++)//we are finished when all bits of the message are looked at
        {
            //crc_bit = (crc >> 15) & 1;//bit of crc we are working on. 15=poly order-1
            //crc <<= 1;//shift to next crc bit (have to do this here before Gate A does its thing)
            //if(crc_bit ^ bits[i])crc = crc ^ 0x1021;//add to the crc the poly mod2 if crc_bit + block_bit = 1 mod2 (0x1021 is the ploy with the first bit missing so this means x^16+x^12+x^5+1)

            //differnt endiness
            crc_bit = crc & 1;//bit of crc we are working on. 15=poly order-1
            crc >>= 1;//shift to next crc bit (have to do this here before Gate A does its thing)
            if(crc_bit ^ bits[i])crc = crc ^ 0x8408;//(0x8408 is reversed 0x1021)add to the crc the poly mod2 if crc_bit + block_bit = 1 mod2 (0x1021 is the ploy with the first bit missing so this means x^16+x^12+x^5+1)

        }
        crc=~crc;
        return crc;
    }
    quint16 calcusingbytes(char *bytes,int numberofbytes)
    {
        crc = 0xFFFF;
        int crc_bit;
        int message_bit;
        int message_byte;
        for(int i=0; i<numberofbytes; i++)//we are finished when all bits of the message are looked at
        {
            message_byte=bytes[i];
            for(int k=0;k<8;k++)
            {

                //message_bit=(message_byte>>7)&1;
                //message_byte<<=1;
                //crc_bit = (crc >> 15) & 1;//bit of crc we are working on. 15=poly order-1
                //crc <<= 1;//shift to next crc bit (have to do this here before Gate A does its thing)
                //if(crc_bit ^ message_bit)crc = crc ^ 0x1021;//add to the crc the poly mod2 if crc_bit + block_bit = 1 mod2 (0x1021 is the ploy with the first bit missing so this means x^16+x^12+x^5+1)

                //differnt endiness
                message_bit=message_byte&1;
                message_byte>>=1;
                crc_bit = crc & 1;//bit of crc we are working on. 15=poly order-1
                crc >>= 1;//shift to next crc bit (have to do this here before Gate A does its thing)
                if(crc_bit ^ message_bit)crc = crc ^ 0x8408;//(0x8408 is reversed 0x1021)add to the crc the poly mod2 if crc_bit + block_bit = 1 mod2 (0x1021 is the ploy with the first bit missing so this means x^16+x^12+x^5+1)

            }
        }
        crc=~crc;
        return crc;
    }
    quint16 calcusingbytes_update(uchar message_byte)
    {
        crc = ~crc;
        int crc_bit;
        int message_bit;

        for(int k=0;k<8;k++)
        {

            //message_bit=(message_byte>>7)&1;
            //message_byte<<=1;
            //crc_bit = (crc >> 15) & 1;//bit of crc we are working on. 15=poly order-1
            //crc <<= 1;//shift to next crc bit (have to do this here before Gate A does its thing)
            //if(crc_bit ^ message_bit)crc = crc ^ 0x1021;//add to the crc the poly mod2 if crc_bit + block_bit = 1 mod2 (0x1021 is the ploy with the first bit missing so this means x^16+x^12+x^5+1)

            //differnt endiness
            message_bit=message_byte&1;
            message_byte>>=1;
            crc_bit = crc & 1;//bit of crc we are working on. 15=poly order-1
            crc >>= 1;//shift to next crc bit (have to do this here before Gate A does its thing)
            if(crc_bit ^ message_bit)crc = crc ^ 0x8408;//(0x8408 is reversed 0x1021)add to the crc the poly mod2 if crc_bit + block_bit = 1 mod2 (0x1021 is the ploy with the first bit missing so this means x^16+x^12+x^5+1)

        }

        crc=~crc;
        return crc;
    }
    quint16 calcusingbytesotherendines(char *bytes,int numberofbytes)
    {
        crc = 0xFFFF;
        int crc_bit;
        int message_bit;
        int message_byte;
        for(int i=0; i<numberofbytes; i++)//we are finished when all bits of the message are looked at
        {
            message_byte=bytes[i];
            for(int k=0;k<8;k++)
            {

                message_bit=(message_byte>>7)&1;
                message_byte<<=1;
                crc_bit = (crc >> 15) & 1;//bit of crc we are working on. 15=poly order-1
                crc <<= 1;//shift to next crc bit (have to do this here before Gate A does its thing)
                if(crc_bit ^ message_bit)crc = crc ^ 0x1021;//add to the crc the poly mod2 if crc_bit + block_bit = 1 mod2 (0x1021 is the ploy with the first bit missing so this means x^16+x^12+x^5+1)

                //differnt endiness
                //message_bit=message_byte&1;
                //message_byte>>=1;
                //crc_bit = crc & 1;//bit of crc we are working on. 15=poly order-1
                //crc >>= 1;//shift to next crc bit (have to do this here before Gate A does its thing)
                //if(crc_bit ^ message_bit)crc = crc ^ 0x8408;//(0x8408 is reversed 0x1021)add to the crc the poly mod2 if crc_bit + block_bit = 1 mod2 (0x1021 is the ploy with the first bit missing so this means x^16+x^12+x^5+1)

            }
        }
        crc=~crc;
        return crc;
    }
    quint16 calcusingbytes(QByteArray data)
    {
        return calcusingbytes(data.data(),data.size());
    }
    quint16 calcusingbytesotherendines(QByteArray data)
    {
        return calcusingbytesotherendines(data.data(),data.size());
    }
};



class TSlip
{
public:
    TSlip();
    QByteArray &EscapePacket(uchar packet_type, const QByteArray &data);
    void NewRxData();
    bool GotRXPacket(const QByteArray &data);

    QByteArray RxPacket;
    uchar RxPacket_type;

    int goodBytes_cnt;
    int badBytes_cnt;

private:
    enum ESC_CHARS {
        END     =   0xC0,    /* indicates end of packet */
        ESC     =   0xDB,    /* indicates byte stuffing */
        ESC_END =   0xDC,    /* ESC ESC_END means END data byte */
        ESC_ESC =   0xDE,    /* ESC ESC_ESC means ESC data byte */
    };
    bool escapeing;
    const QByteArray *privateRXdataPtr;
    int rxi;
    bool LastGotRXPacketWasTrue;

    QByteArray tmppkt;

    CRC16 crc16;
};


class DSCADataDeFormatterScrambler
{
public:
    DSCADataDeFormatterScrambler()
    {
        reset_act();
    }
    void reset()
    {
        reset_req=true;
    }
    void update(QVector<int> &data)
    {
        if(reset_req)reset_act();
        for(int j=0;j<data.size();j++)
        {
            int val0=state[0]^state[14];
            data[j]^=val0;
            for(int i=state.size()-1;i>0;i--)state[i]=state[i-1];
            state[0]=val0;
        }
    }
private:
    void reset_act()
    {
        int tmp[]={1,1,0,1,0,0,1,0,1,0,1,1,0,0,1,-1};
        state.clear();
        for(int i=0;tmp[i]>=0;i++)state.push_back(tmp[i]);
        reset_req=false;
    }
    QVector<int> state;
    bool reset_req;
};

class DelayLine
{
public:
    DelayLine()
    {
        setLength(12);
    }
    void setLength(int length)
    {
        length++;
        assert(length>0);
        buffer.resize(length);
        buffer_ptr=0;
        buffer_sz=buffer.size();
    }
    void update(QVector<int> &data)
    {
        for(int i=0;i<data.size();i++)
        {
            buffer[buffer_ptr]=data[i];
            buffer_ptr++;buffer_ptr%=buffer_sz;
            data[i]=buffer[buffer_ptr];
        }
    }
private:
    QVector<int> buffer;
    int buffer_ptr;
    int buffer_sz;
};

template<typename T>
class DSCADataDeFormatterInterleaver
{
public:
    DSCADataDeFormatterInterleaver();
    void setColumnSize(int N);
    QVector<T> &interleave(QVector<T> &block);
    QVector<T> &deinterleave(QVector<T> &block);
    QVector<T> &deinterleave(QVector<T> &block,int cols);//deinterleaves with a fewer number of cols than the block has
    QByteArray &deinterleave_ba(QVector<T> &block);//if the user wants a byte array output (only works for uchar and char)
    QSize getSize();
private:
    QVector<T> matrix;
    QByteArray matrix_ba;
    int M;
    int N;
    QVector<int> interleaverowpermute;
    QVector<int> interleaverowdepermute;
};

class PreambleDetector
{
public:
    PreambleDetector();
    void setPreamble(QVector<int> _preamble);
    bool setPreamble(quint64 bitpreamble,int len);
    bool Update(int val);
private:
    QVector<int> preamble;
    QVector<int> buffer;
    int buffer_ptr;
};

class PreambleDetectorPhaseInvariant
{
public:
    PreambleDetectorPhaseInvariant();
    void setPreamble(QVector<int> _preamble);
    bool setPreamble(quint64 bitpreamble,int len);
    void setTollerence(int tollerence);
    int Update(int val);
    bool inverted;
private:
    QVector<int> preamble;
    QVector<int> buffer;
    int buffer_ptr;
    int tollerence;
};

class OQPSKPreambleDetectorAndAmbiguityCorrection
{
public:
    OQPSKPreambleDetectorAndAmbiguityCorrection();
    void setPreamble(QVector<int> _preamble);
    bool setPreamble(quint64 bitpreamble,int len);
    bool Update(int val);
private:
    QVector<int> preamble;
    QVector<int> buffer;
    int buffer_ptr;
};

class DSCADataDeFormatter : public QObject
{
    Q_OBJECT
public:
    enum Mode {none=-1,mode0=0,mode1=1,mode2=2,mode3=3,mode4=4};
    typedef enum ModeCodes //hamming distance of 8
    {
        mode_code_15=0b1000000000000000,
        mode_code_14=0b0000000011111111,
        mode_code_13=0b1000111100001111,
        mode_code_12=0b0000111111110000,
        mode_code_11=0b1011001100110011,
        mode_code_10=0b0011001111001100,
        mode_code_9=0b1011110000111100,
        mode_code_8=0b0011110011000011,
        mode_code_7=0b1101010101010101,
        mode_code_6=0b0101010110101010,
        mode_code_5=0b1101101001011010,
        mode_code_4=0b0101101010100101,
        mode_code_3=0b1110011001100110,
        mode_code_2=0b0110011010011001,
        mode_code_1=0b1110100101101001,
        mode_code_0=0b0110100110010110
    }ModeCodes;

    enum PacketType {PacketType_RDS=0,PacketType_OPUS=1};

    explicit DSCADataDeFormatter(QObject *parent = 0);
    ~DSCADataDeFormatter();

signals:
    void DataCarrierDetect(bool status);
    void Errorsignal(QString &error);
    void signalDSCAPacket(uchar packet_type, QByteArray packet) const;
    void signalDSCAOpusPacket(QByteArray packet) const;
    void signalDSCARDSPacket(QByteArray packet) const;
    void signalModeChanged(DSCADataDeFormatter::Mode mode);
    void ShortDataFrame();
    void signalBER(double ber);
public slots:
    void setMode(DSCADataDeFormatter::Mode mode);
    void setBitRate(double fb);
    void setSettings(double fb,DSCADataDeFormatter::Mode mode);
    void SignalStatusSlot(bool signal)
    {
        if(!signal)LostSignal();
    }
    void LostSignal()
    {
        cntr=1000000000;
        datacdcountdown=0;
        datacd=false;
        emit DataCarrierDetect(false);
    }
    void enableHardFECDecoderType(bool enable)
    {
        FEC_hard_Type=enable;
        //qDebug()<<"FEC_hard_Type"<<enable;
    }
    void processDemodulatedSoftBits(const QByteArray &soft_bits);
private:

    QVector<short> sbits;
    QByteArray decodedbytes;
    PreambleDetector preambledetector;

    Mode mode;
    QVector<int> modecode_counts;

    double ber;

    int ifb;
    double fb;

    //

    //OQPSK
    PreambleDetectorPhaseInvariant preambledetectorphaseinvariantimag;
    PreambleDetectorPhaseInvariant preambledetectorphaseinvariantreal;

    bool useingOQPSK;
    int NumberOfBitsInBlock;//info only
    int NumberOfBitsInFrame;//info and header and uw
    int NumberOfBitsInHeader;
    int realimag;



    QVector<char> block;
    DSCADataDeFormatterInterleaver<char> leaver;
    DSCADataDeFormatterScrambler scrambler;


    JConvolutionalCodec *jconvolcodec;//new soft
    bool FEC_hard_Type;//just so I can see the difference a soft decoder makes

    PuncturedCode pc;


    DelayLine dl1,dl2;

    quint16 frameinfo;

    QByteArray infofield;


    int datacdcountdown;
    bool datacd;
    int cntr;


    int gotsync_last;
    int blockcnt;

    //opus
    TSlip slip;



private slots:
    void updateDCD();

};

#endif // DSCADataDeFormatter_H
