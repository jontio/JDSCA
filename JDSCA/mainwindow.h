#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include "audiooqpskdemodulator.h"
#include "gui_classes/settingsdialog.h"
#include "dscadatadeformatter.h"
#include "opusaudioout.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    enum DemodType{NoDemodType,OQPSK};
    Ui::MainWindow *ui;
    QLabel *ebnolabel;
    QLabel *freqlabel;
    QLabel *modelabel;
    QLabel *berlabel;
    QLabel *bitratelabel;

    //OQPSK demodulator
    AudioOqpskDemodulator *audiooqpskdemodulator;
    AudioOqpskDemodulator::Settings audiooqpskdemodulatorsettings;
    //

    DSCADataDeFormatter *datadeformatter;
    OpusAudioOut *opusAudioOut;

    SettingsDialog *settingsdialog;

    void acceptsettings();

    QTimer *RDStimeout;


protected:
    void closeEvent(QCloseEvent *event);


private slots:
    void DataCarrierDetectStatusSlot(bool dcd);
    void SignalStatusSlot(bool signal);
    void OpusSignalStatusSlot(bool signal);
    void EbNoSlot(double EbNo);
    void WarningTextSlot(QString warning);
    void DSCARDSPacketSlot(QByteArray &ba);
    void PeakVolumeSlot(double Volume);
    void bitRateChangedSlot(double bitrate);
    void PlottablesSlot(double freq_est,double freq_center,double bandwidth);
    void on_BERstatus(double ber);
    void AboutSlot();
    void on_comboBoxdisplay_currentIndexChanged(const QString &arg1);
    void on_action_Settings_triggered();
    void ERRorslot(QString &error);
    void on_modeChanged(DSCADataDeFormatter::Mode mode);
    void RDStimeoutSlot();
    void on_actionSpectrum_triggered();
    void on_actionConstellation_triggered();
    void srinkwindow();
    void on_actionAbout_Qt_triggered();
    void on_horizontalSlider_bw_sliderMoved(int position);
};

#endif // MAINWINDOW_H
