#include "settingsdialog.h"
#include "ui_settingsdialog.h"
#include <QDebug>
#include <QSettings>
#include <QStandardPaths>
#include <QFile>
#include <QMessageBox>
#include "JSound.h"
#include "sdr.h"


SettingsDialog::SettingsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SettingsDialog)
{
    ui->setupUi(this);
    ui->comboBoxsoundcard->addItem("RTL SDR");
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

    rtl_device_index=ui->comboRTLDevices->currentIndex();
    rtl_afc=ui->checkBox_rtl_afc->isChecked();
    rtl_agc=ui->checkBox_rtl_agc->isChecked();
    rtl_subcarrier_freq_offset=(ui->doubleSpinBox_rtl_subcarrier_freq_offset->value()*1000.0);//from kHz to Hz
    rtl_gain=ui->doubleSpinBox_rtl_gain->value();

}


void SettingsDialog::populatesettings()
{

    //rtaudio
    //on Raspberry and ALSA the devices cant always seem to be enumerated more than once so I cant clear these combo boxes just in case
    //ui->comboBoxsoundcard_out->clear();
    //ui->comboBoxsoundcard->clear();
    //ui->comboBoxsoundcard->addItem("RTL SDR")
    TJCSound jcsound;
    SDevices devices=jcsound.Devices;
    for(unsigned int i=0;i<devices.NumberOfDevices;i++)
    {
        if(devices.Device[i].outchannelcount>0&&ui->comboBoxsoundcard_out->findText(devices.Device[i].name)<0)ui->comboBoxsoundcard_out->addItem(devices.Device[i].name);
        if(devices.Device[i].inchannelcount>0&&ui->comboBoxsoundcard->findText(devices.Device[i].name)<0)ui->comboBoxsoundcard->addItem(devices.Device[i].name);

    }

    //rtl
    ui->comboRTLDevices->clear();
    ui->comboRTLDevices->addItems(SDR_Enum::DeviceNames());

    //load settings
    QSettings settings("Jontisoft", "JDSCA");
    ui->comboBoxsoundcard->setCurrentText(settings.value("comboBoxsoundcard","").toString());
    ui->comboBoxsoundcard_out->setCurrentText(settings.value("comboBoxsoundcard_out","").toString());

    ui->comboBox_fec_type->setCurrentIndex(settings.value("comboBox_fec_type",0).toInt());
    ui->checkBox_cma->setChecked(settings.value("checkBox_cma",false).toBool());
    ui->checkBox_tracking->setChecked(settings.value("checkBox_tracking",false).toBool());

    //RTL
    ui->comboRTLDevices->setCurrentIndex(settings.value("comboRTLDevices",0).toInt());
    ui->checkBox_rtl_afc->setChecked(settings.value("checkBox_rtl_afc",true).toBool());
    ui->checkBox_rtl_agc->setChecked(settings.value("checkBox_rtl_agc",true).toBool());
    ui->doubleSpinBox_rtl_subcarrier_freq_offset->setValue(settings.value("doubleSpinBox_rtl_subcarrier_freq_offset",0.0).toDouble());
    ui->doubleSpinBox_rtl_gain->setValue(settings.value("doubleSpinBox_rtl_gain",50).toDouble());


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

    //RTL
    settings.setValue("comboRTLDevices",ui->comboRTLDevices->currentIndex());
    settings.setValue("checkBox_rtl_afc",ui->checkBox_rtl_afc->isChecked());
    settings.setValue("checkBox_rtl_agc",ui->checkBox_rtl_agc->isChecked());
    settings.setValue("doubleSpinBox_rtl_subcarrier_freq_offset",ui->doubleSpinBox_rtl_subcarrier_freq_offset->value());
    settings.setValue("doubleSpinBox_rtl_gain",ui->doubleSpinBox_rtl_gain->value());

    poulatepublicvars();
    QDialog::accept();
}


