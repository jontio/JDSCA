#ifndef AUDIOOQPSKDEMODULATOR_H
#define AUDIOOQPSKDEMODULATOR_H

#include <QObject>
#include <QAudioInput>
#include <QMutex>

#include "oqpskdemodulator.h"

class AudioOqpskDemodulator : public OqpskDemodulator
{
    Q_OBJECT
public:
    struct Settings : public OqpskDemodulator::Settings
    {
        QAudioDeviceInfo audio_device_in;
        double buffersizeinsecs;
        Settings()
        {
            audio_device_in=QAudioDeviceInfo::defaultInputDevice();
            buffersizeinsecs=1.0;
        }
    };
    explicit AudioOqpskDemodulator(QObject *parent = 0);
    ~AudioOqpskDemodulator();
    void start();
    void stop();
    void setSettings(Settings settings);
private:
    Settings settings;
    QAudioFormat m_format;
    QAudioInput *m_audioInput;

    QTimer *settingapplytimeout;
    Settings timeout_settings;
    bool timeout_wasopen;

private slots:
    void setSettings_apply();
};

#endif // AUDIOOQPSKDEMODULATOR_H
