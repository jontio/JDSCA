#ifndef SETTINGSDIALOG_H
#define SETTINGSDIALOG_H

#include <QtGlobal>
#include "sdr.h"


#if defined(Q_OS_UNIX) || defined(Q_OS_LUNX)
#define APPDATALOCATIONS QStandardPaths::AppDataLocation
#else
#define APPDATALOCATIONS QStandardPaths::AppLocalDataLocation
#endif

#include <QDialog>
#include <QVector>


namespace Ui {
class SettingsDialog;
}

class SettingsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SettingsDialog(QWidget *parent = 0);
    ~SettingsDialog();
    void populatesettings();

    bool dropnontextmsgs;
    QString msgdisplayformat;
    bool widebandwidthenable;


    bool beepontextmessage;

    QString WantedOutSoundDevice;

    QString WantedInSoundDevice;

    bool use_cma;
    bool use_tracking;
    bool use_hard_decoding;

    int rtl_device_index;
    bool rtl_afc;
    bool rtl_agc;
    int rtl_subcarrier_freq_offset;//in Hz
    double rtl_gain;
    SDR::Filter_selection rtl_filter_selection;

private:
    Ui::SettingsDialog *ui;    
    void poulatepublicvars();

protected:
    void accept();

private slots:




};

#endif // SETTINGSDIALOG_H
