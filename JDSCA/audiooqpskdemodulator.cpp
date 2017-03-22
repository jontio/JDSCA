#include "audiooqpskdemodulator.h"

#include <QDebug>

AudioOqpskDemodulator::AudioOqpskDemodulator(QObject *parent)
:   OqpskDemodulator(parent),
  m_audioInput(NULL)
{
    settingapplytimeout=new QTimer(this);
    connect(settingapplytimeout,SIGNAL(timeout()),this,SLOT(setSettings_apply()));
}

void AudioOqpskDemodulator::start()
{
    if(settingapplytimeout->isActive())
    {
        timeout_wasopen=true;
        return;
    }
    OqpskDemodulator::start();
    if(m_audioInput)m_audioInput->start(this);
}

void AudioOqpskDemodulator::stop()
{
    if(settingapplytimeout->isActive())
    {
        timeout_wasopen=false;
        return;
    }
    if(m_audioInput)m_audioInput->stop();
    OqpskDemodulator::stop();
}

void AudioOqpskDemodulator::setSettings(Settings _settings)
{
    timeout_settings=_settings;
    if(!settingapplytimeout->isActive())
    {
        timeout_wasopen=isOpen();
        if(isOpen())
        {
            if(m_audioInput)m_audioInput->stop();
            OqpskDemodulator::stop();
        }
    }
    settingapplytimeout->stop();
    settingapplytimeout->start(500);
}

void AudioOqpskDemodulator::setSettings_apply()
{
    settingapplytimeout->stop();
    bool wasopen=timeout_wasopen;

    Settings _settings=timeout_settings;

    //if Fs has changed or the audio device doesnt exist or the input device has changed then need to redo the audio device
    if((_settings.Fs!=settings.Fs)||(!m_audioInput)||(_settings.audio_device_in!=settings.audio_device_in))
    {
        settings=_settings;

        if(m_audioInput)m_audioInput->deleteLater();

        //set the format
        m_format.setSampleRate(settings.Fs);
        m_format.setChannelCount(1);
        m_format.setSampleSize(16);
        m_format.setCodec("audio/pcm");
        m_format.setByteOrder(QAudioFormat::LittleEndian);
        m_format.setSampleType(QAudioFormat::SignedInt);

        //setup
        m_audioInput = new QAudioInput(settings.audio_device_in, m_format, this);
        m_audioInput->setBufferSize(settings.Fs*settings.buffersizeinsecs);//buffersizeinsecs seconds of buffer
    }
    settings=_settings;

    OqpskDemodulator::setSettings(settings);

    if(wasopen)start();

}

AudioOqpskDemodulator::~AudioOqpskDemodulator()
{
    stop();
}
