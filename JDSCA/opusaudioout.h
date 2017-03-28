#ifndef OPUSAUDIOOUT_H
#define OPUSAUDIOOUT_H

#include <QObject>
#include "opus/opus.h"
#include "JSound.h"
#include "DSP.h"
#include <QMutex>
#include <QTimer>


class OpusAudioOut : public QObject
{
    Q_OBJECT
public:
    OpusAudioOut(QObject *parent = 0);
    ~OpusAudioOut();
    void set_soundcard_name(QString &name);
    QString get_soundcard_name();
signals:
    void OpusDCDchanged(bool opusdcd);
public slots:
    void decode(const QByteArray &packet);

    void SoundEvent(qint16 *inputBuffer, qint16 *outputBuffer, int nBufferFrames);
private:

    #define CHANNELS 1
    #define SAMPLE_RATE 48000
    #define MAX_FRAME_SIZE 6*960
    OpusDecoder *decoder;
    opus_int16 out[MAX_FRAME_SIZE*CHANNELS];

//    #define FRAME_SIZE 960
//    //#define APPLICATION OPUS_APPLICATION_VOIP
//    #define APPLICATION OPUS_APPLICATION_AUDIO
//    #define BITRATE 8000
//    #define MAX_PACKET_SIZE (3*1276)
//    opus_int16 in[FRAME_SIZE*CHANNELS];
//    unsigned char cbits[MAX_PACKET_SIZE];
//    int nbBytes;

    TJCSound *jsound;

    QMutex callback_mutex;
    QVector<qint16> opus_buffer;
    int opus_buf_ptr_head;
    int opus_buf_ptr_tail;
    double opus_rate;
    double opus_ratechange;
    double opus_buffer_use_percentage;

    bool opus_buffer_empty;
    bool opus_buffer_empty_last;
    bool opus_signal;
    qint16 opus_val;

    qint32 samples_received;
    qint32 samples_played;
    double current_number_of_samples_to_take;

    QTimer *infotimer;

private slots:

    void on_infotimer();



};

#endif // OPUSAUDIOOUT_H
