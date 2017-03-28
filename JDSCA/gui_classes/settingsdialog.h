#ifndef SETTINGSDIALOG_H
#define SETTINGSDIALOG_H

#include <QtGlobal>


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

private:
    Ui::SettingsDialog *ui;    
    void poulatepublicvars();

protected:
    void accept();

private slots:




};

#endif // SETTINGSDIALOG_H
