#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QStandardPaths>

#ifdef __linux__
#include <unistd.h>
#define Sleep(x) usleep(x * 1000);
#endif
#ifdef _WIN32
#include <windows.h>
#endif



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);



    srinkwindow();

    setFixedSize(width(),height());

    //create areo decoder
    datadeformatter = new DSCADataDeFormatter(this); //Create datadeformatter

    //create opus audio out decoder
    opusAudioOut = new OpusAudioOut(this);

    //create settings dialog.
    settingsdialog = new SettingsDialog(this);

    //create the oqpsk demodulator
    audiooqpskdemodulator = new AudioOqpskDemodulator(this);

    //default sink is the datadeformatter device
    audiooqpskdemodulator->ConnectSinkDevice(datadeformatter);

    //console setup
    ui->console->setEnabled(true);
    ui->console->setLocalEchoEnabled(true);

    //datadeformatter setup
    datadeformatter->ConnectSinkDevice(ui->console->consoledevice);

    //statusbar setup
    berlabel = new QLabel();
    freqlabel = new QLabel();
    ebnolabel = new QLabel();
    modelabel = new QLabel();
    bitratelabel = new QLabel();
    ui->statusBar->addPermanentWidget(new QLabel());
    ui->statusBar->addPermanentWidget(modelabel);
    ui->statusBar->addPermanentWidget(bitratelabel);
    ui->statusBar->addPermanentWidget(freqlabel);
    ui->statusBar->addPermanentWidget(berlabel);
    ui->statusBar->addPermanentWidget(ebnolabel);

    //led setup
    ui->ledvolume->setLED(QIcon::Off);
    ui->ledsignal->setLED(QIcon::Off);
    ui->ledopus->setLED(QIcon::Off);

    //misc connections
    connect(ui->action_About,    SIGNAL(triggered()),                                   this, SLOT(AboutSlot()));

    //oqpsk connections
    connect(audiooqpskdemodulator, SIGNAL(Plottables(double,double,double)),              this,SLOT(PlottablesSlot(double,double,double)));
    connect(audiooqpskdemodulator, SIGNAL(SignalStatus(bool)),                            this,SLOT(SignalStatusSlot(bool)));
    connect(audiooqpskdemodulator, SIGNAL(WarningTextSignal(QString)),                    this,SLOT(WarningTextSlot(QString)));
    connect(audiooqpskdemodulator, SIGNAL(EbNoMeasurmentSignal(double)),                  this,SLOT(EbNoSlot(double)));
    connect(audiooqpskdemodulator, SIGNAL(PeakVolume(double)),                            this, SLOT(PeakVolumeSlot(double)));
    connect(audiooqpskdemodulator, SIGNAL(OrgOverlapedBuffer(QVector<double>)),           ui->spectrumdisplay,SLOT(setFFTData(QVector<double>)));
    connect(audiooqpskdemodulator, SIGNAL(Plottables(double,double,double)),              ui->spectrumdisplay,SLOT(setPlottables(double,double,double)));
    connect(audiooqpskdemodulator, SIGNAL(SampleRateChanged(double)),                     ui->spectrumdisplay,SLOT(setSampleRate(double)));
    connect(audiooqpskdemodulator, SIGNAL(ScatterPoints(QVector<cpx_type>)),              ui->scatterplot,SLOT(setData(QVector<cpx_type>)));
    connect(ui->spectrumdisplay,   SIGNAL(CenterFreqChanged(double)),                     audiooqpskdemodulator,SLOT(CenterFreqChangedSlot(double)));
    connect(audiooqpskdemodulator, SIGNAL(BitRateChanged(double)),                        datadeformatter,SLOT(setBitRate(double)));
    connect(audiooqpskdemodulator, SIGNAL(BitRateChanged(double)),                        this,SLOT(bitRateChangedSlot(double)));
    connect(audiooqpskdemodulator, SIGNAL(SignalStatus(bool)),                            datadeformatter,SLOT(SignalStatusSlot(bool)));

    //datadeformatter connections
    connect(datadeformatter,SIGNAL(DataCarrierDetect(bool)),this,SLOT(DataCarrierDetectStatusSlot(bool)));
    connect(datadeformatter,SIGNAL(DataCarrierDetect(bool)),audiooqpskdemodulator,SLOT(DCDstatSlot(bool)));
    connect(datadeformatter,SIGNAL(signalDSCAOpusPacket(QByteArray&)),opusAudioOut,SLOT(decode(QByteArray&)));
    connect(datadeformatter,SIGNAL(signalModeChanged(DSCADataDeFormatter::Mode)),this,SLOT(on_modeChanged(DSCADataDeFormatter::Mode)));
    connect(datadeformatter,SIGNAL(signalBER(double)),this,SLOT(on_BERstatus(double)));
    connect(datadeformatter,SIGNAL(signalDSCARDSPacket(QByteArray&)),this,SLOT(DSCARDSPacketSlot(QByteArray&)));

    //load settings
    QSettings settings("Jontisoft", "JDSCA");
    ui->comboBoxdisplay->setCurrentIndex(settings.value("comboBoxdisplay",0).toInt());
    ui->horizontalSlider_bw->setValue(settings.value("bandwidth",10000).toDouble());
    double tmpfreq=settings.value("freq_center",1000).toDouble(); 
    double tmpbandwidth=settings.value("bandwidth",10000).toDouble();
    if(!settings.value("frame_spectrum",1).toBool())QTimer::singleShot(0,this,SLOT(on_actionSpectrum_triggered()));
    if(!settings.value("groupBox_scatterplot",1).toBool())QTimer::singleShot(0,this,SLOT(on_actionConstellation_triggered()));

    //set audio demodulator settings and start
    on_comboBoxdisplay_currentIndexChanged(ui->comboBoxdisplay->currentText());
    on_horizontalSlider_bw_sliderMoved(ui->horizontalSlider_bw->value());

    //oqpsk
    audiooqpskdemodulatorsettings.Fs=192000;
    audiooqpskdemodulatorsettings.audio_device_in=settingsdialog->audioinputdevice;
    audiooqpskdemodulatorsettings.freq_center=tmpfreq;
    audiooqpskdemodulatorsettings.lockingbw=tmpbandwidth;
    audiooqpskdemodulator->setSettings(audiooqpskdemodulatorsettings);
    audiooqpskdemodulator->start();

    //Opusaudioout connections
    connect(opusAudioOut,SIGNAL(OpusDCDchanged(bool)),this,SLOT(OpusSignalStatusSlot(bool)));

    //set pop and accept settings
    settingsdialog->populatesettings();
    acceptsettings();

    //RDS display timeout
    RDStimeout = new QTimer(this);
    connect(RDStimeout,SIGNAL(timeout()),this,SLOT(RDStimeoutSlot()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{

    //save settings
    QSettings settings("Jontisoft", "JDSCA");
    settings.setValue("bandwidth", audiooqpskdemodulator->getBandwidth());
    settings.setValue("comboBoxdisplay", ui->comboBoxdisplay->currentIndex());
    settings.setValue("freq_center", audiooqpskdemodulator->getCurrentFreq());
    settings.setValue("frame_spectrum", ui->frame_spectrum->isVisible());
    settings.setValue("groupBox_scatterplot", ui->groupBox_scatterplot->isVisible());

    event->accept();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::OpusSignalStatusSlot(bool signal)
{
    if(signal)ui->ledopus->setLED(QIcon::On);
     else ui->ledopus->setLED(QIcon::Off);
}

void MainWindow::SignalStatusSlot(bool signal)
{
    if(signal)ui->ledsignal->setLED(QIcon::On);
     else ui->ledsignal->setLED(QIcon::Off);
}

void MainWindow::DataCarrierDetectStatusSlot(bool dcd)
{
    if(dcd)ui->leddata->setLED(QIcon::On);
     else {ui->leddata->setLED(QIcon::Off);ui->label_rds->clear();}
}

//guess EbNo using MER
void MainWindow::EbNoSlot(double EbNo)
{
    ebnolabel->setText(((QString)" MER: %1dB ").arg(qRound(round(EbNo)),2, 10, QChar(' ')));
}

void MainWindow::WarningTextSlot(QString warning)
{
    ui->statusBar->showMessage("Warning: "+warning,5000);
}

void MainWindow::PeakVolumeSlot(double Volume)
{
    if(Volume>0.9)ui->ledvolume->setLED(QIcon::On,QIcon::Active);
     else if(Volume<0.1)ui->ledvolume->setLED(QIcon::Off);
      else ui->ledvolume->setLED(QIcon::On);
}

void MainWindow::bitRateChangedSlot(double bitrate)
{
    bitratelabel->setText(((QString)" %1 bps  ").arg(bitrate));
}

void MainWindow::PlottablesSlot(double freq_est,double freq_center,double bandwidth)
{
    Q_UNUSED(freq_center);
    Q_UNUSED(bandwidth);
    QString str=(((QString)"%1Hz  ").arg(freq_est,0, 'f', 2)).rightJustified(11,' ');
    freqlabel->setText("  Freq: "+str);
}

void MainWindow::on_BERstatus(double ber)
{
    QString str=(((QString)"%1% ").arg(ber*100.0,0, 'f', 0)).rightJustified(6,' ');
    berlabel->setText("  BER: "+str);
    //berlabel->setText(((QString)" BER: %1% ").arg((int)round(ber*100),2, 10, QChar('0')));
}

void MainWindow::AboutSlot()
{
    QMessageBox::about(this,"JDSCA",""
                                     "<H1>A DSCA demodulator and decoder</H1>"
                                     "<H3>v1</H3>"
                                     "<p>This is a program to listen to digital SCA signals.</p>"
                                     "<p><a href=\"http://jontio.zapto.org/\">http://jontio.zapto.org/</a>.</p>"
                                     "<p>Jonti 2017</p>" );


}

void MainWindow::on_modeChanged(DSCADataDeFormatter::Mode mode)
{


    switch(mode)
    {
    case DSCADataDeFormatter::mode0:
        modelabel->setText("  mode0  ");
        break;
    case DSCADataDeFormatter::mode1:
        modelabel->setText("  mode1  ");
        break;
    case DSCADataDeFormatter::mode2:
        modelabel->setText("  mode2  ");
        break;
    case DSCADataDeFormatter::mode3:
        modelabel->setText("  mode3  ");
        break;
    case DSCADataDeFormatter::mode4:
        modelabel->setText("  mode4  ");
        break;
    default:
        modelabel->setText("  none   ");
    break;
    }

}

void MainWindow::on_comboBoxdisplay_currentIndexChanged(const QString &arg1)
{
    ui->scatterplot->graph(0)->clearData();
    ui->scatterplot->replot();
    ui->scatterplot->setDisksize(3);
    if(arg1=="Constellation")
    {
        audiooqpskdemodulator->setScatterPointType(OqpskDemodulator::SPT_constellation);
    }
    if(arg1=="Constellation 8")
    {
        audiooqpskdemodulator->setScatterPointType(OqpskDemodulator::SPT_8constellation);
    }
    if(arg1=="Symbol phase")
    {
        audiooqpskdemodulator->setScatterPointType(OqpskDemodulator::SPT_phaseoffsetest);
        ui->scatterplot->setDisksize(6);
    }
    if(arg1=="None")
    {
        audiooqpskdemodulator->setScatterPointType(OqpskDemodulator::SPT_None);
    }
}

void MainWindow::on_action_Settings_triggered()
{
    settingsdialog->populatesettings();
    if(settingsdialog->exec()==QDialog::Accepted)acceptsettings();
}

void MainWindow::acceptsettings()
{

    ui->statusBar->clearMessage();
    opusAudioOut->set_soundcard_name(settingsdialog->WantedOutSoundDevice);
    if(audiooqpskdemodulatorsettings.audio_device_in!=settingsdialog->audioinputdevice)
    {
        audiooqpskdemodulator->stop();
        audiooqpskdemodulatorsettings.audio_device_in=settingsdialog->audioinputdevice;
        audiooqpskdemodulatorsettings.freq_center=audiooqpskdemodulator->getCurrentFreq();
        audiooqpskdemodulatorsettings.lockingbw=audiooqpskdemodulator->getBandwidth();
        audiooqpskdemodulator->setSettings(audiooqpskdemodulatorsettings);
        audiooqpskdemodulator->start();
    }

    audiooqpskdemodulator->setCMA(settingsdialog->use_cma);
    datadeformatter->enableHardFECDecoderType(settingsdialog->use_hard_decoding);
    audiooqpskdemodulator->setAFC(settingsdialog->use_tracking);


}

void MainWindow::ERRorslot(QString &error)
{
    ui->console->appendHtml("<font color=\"red\">"+error+"</font>");
}

void MainWindow::DSCARDSPacketSlot(QByteArray &ba)
{
    if(ba.size()<2)return;
    QString PS=ba.mid(1,ba[0]);
    QString RT=ba.mid(ba[0]+2);
    ui->label_rds->setText("<h2>"+PS+"</h2>"+RT);
    RDStimeout->start(10000);
}

void MainWindow::RDStimeoutSlot()
{
    ui->label_rds->clear();
    RDStimeout->stop();
}

void MainWindow::on_actionSpectrum_triggered()
{
    if(ui->frame_spectrum->isVisible())
    {
        ui->frame_spectrum->setVisible(false);
        ui->horizontalSlider_bw->setVisible(false);
        if(!ui->comboBoxdisplay->isVisible())ui->widget_bar->setVisible(false);
        //come on qt this is nasty
        QTimer::singleShot(50,this,SLOT(srinkwindow()));
        QApplication::processEvents( QEventLoop::ExcludeUserInputEvents );
    }
     else
     {
        ui->frame_spectrum->setVisible(true);
        ui->widget_bar->setVisible(true);
        ui->horizontalSlider_bw->setVisible(true);
     }


}

void MainWindow::on_actionConstellation_triggered()
{
    if(ui->groupBox_scatterplot->isVisible())
    {
        ui->groupBox_scatterplot->setVisible(false);
        ui->comboBoxdisplay->setVisible(false);
        if(!ui->frame_spectrum->isVisible())ui->widget_bar->setVisible(false);
        //come on qt this is nasty
        QTimer::singleShot(50,this,SLOT(srinkwindow()));
        QApplication::processEvents( QEventLoop::ExcludeUserInputEvents );
    }
     else
     {
        ui->groupBox_scatterplot->setVisible(true);
        ui->comboBoxdisplay->setVisible(true);
        ui->widget_bar->setVisible(true);
     }
}

void MainWindow::srinkwindow()
{
    resize(0,0);
}

void MainWindow::on_actionAbout_Qt_triggered()
{
    QApplication::aboutQt();
}

void MainWindow::on_horizontalSlider_bw_sliderMoved(int position)
{
    audiooqpskdemodulator->setBandwidth(position);
}
