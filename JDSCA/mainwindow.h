#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QDoubleSpinBox>
#include "oqpskdemodulator.h"
#include "gui_classes/settingsdialog.h"
#include "dscadatadeformatter.h"
#include "opusaudioout.h"
#include "JSound.h"
#include "sdr.h"

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
    QLabel *rtl_freq_label;
    QDoubleSpinBox *rtl_freq_spin;

    //single audio device, one demodulator and some demod settings
    TJCSound *jsound;
    OqpskDemodulator *oqpskdemod;
    OqpskDemodulator::Settings oqpskdemodulatorsettings;

    DSCADataDeFormatter *datadeformatter;
    OpusAudioOut *opusAudioOut;

    SettingsDialog *settingsdialog;

    void acceptsettings();

    QTimer *RDStimeout;

    SDR *sdr;

    QTimer *RTLFreqtimeout;


protected:
    void closeEvent(QCloseEvent *event);


private slots:
    void DataCarrierDetectStatusSlot(bool dcd);
    void SignalStatusSlot(bool signal);
    void OpusSignalStatusSlot(bool signal);
    void EbNoSlot(double EbNo);
    void WarningTextSlot(QString warning);
    void DSCARDSPacketSlot(const QByteArray &ba);
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


    void on_doubleSpinBox_rtl_freq_valueChanged(double arg1);

    void RTL_status_timeout();
    void RTL_freq_timeout();

signals:

};

#endif // MAINWINDOW_H
