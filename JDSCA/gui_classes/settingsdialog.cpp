#include "settingsdialog.h"
#include "ui_settingsdialog.h"
#include <QDebug>
#include <QSettings>
#include <QStandardPaths>
#include <QFile>
#include <QMessageBox>
#include "JSound.h"


SettingsDialog::SettingsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SettingsDialog)
{
    ui->setupUi(this);
    populatesettings();
}

SettingsDialog::~SettingsDialog()
{
    delete ui;
}

void SettingsDialog::poulatepublicvars()
{

    //in
    WantedInSoundDevice=ui->comboBoxsoundcard->currentText();

    //out
    WantedOutSoundDevice=ui->comboBoxsoundcard_out->currentText();



    use_cma=ui->checkBox_cma->isChecked();
    use_tracking=ui->checkBox_tracking->isChecked();
    if(ui->comboBox_fec_type->currentText().toUpper().contains("HARD"))use_hard_decoding=true;
     else use_hard_decoding=false;
}


void SettingsDialog::populatesettings()
{

    //rtaudio
    ui->comboBoxsoundcard_out->clear();
    ui->comboBoxsoundcard->clear();
    TJCSound jcsound;
    SDevices devices=jcsound.Devices;
    for(unsigned int i=0;i<devices.NumberOfDevices;i++)
    {
        if(devices.Device[i].outchannelcount>0)ui->comboBoxsoundcard_out->addItem(devices.Device[i].name);
        if(devices.Device[i].inchannelcount>0)ui->comboBoxsoundcard->addItem(devices.Device[i].name);
    }

    //load settings
    QSettings settings("Jontisoft", "JDSCA");
    ui->comboBoxsoundcard->setCurrentText(settings.value("comboBoxsoundcard","").toString());
    ui->comboBoxsoundcard_out->setCurrentText(settings.value("comboBoxsoundcard_out","").toString());

    ui->comboBox_fec_type->setCurrentIndex(settings.value("comboBox_fec_type",0).toInt());
    ui->checkBox_cma->setChecked(settings.value("checkBox_cma",false).toBool());
    ui->checkBox_tracking->setChecked(settings.value("checkBox_tracking",false).toBool());




    poulatepublicvars();
}

void SettingsDialog::accept()
{    

    //save settings
    QSettings settings("Jontisoft", "JDSCA");

    settings.setValue("comboBoxsoundcard",ui->comboBoxsoundcard->currentText());
    settings.setValue("comboBoxsoundcard_out",ui->comboBoxsoundcard_out->currentText());

    settings.setValue("comboBox_fec_type",ui->comboBox_fec_type->currentIndex());
    settings.setValue("checkBox_cma",ui->checkBox_cma->isChecked());
    settings.setValue("checkBox_tracking",ui->checkBox_tracking->isChecked());


    poulatepublicvars();
    QDialog::accept();
}


