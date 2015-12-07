#include "sensorvisualizer.h"
#include "ui_sensorvisualizer.h"

SensorVisualizer::SensorVisualizer(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SensorVisualizer)
{
    ui->setupUi(this);
}

SensorVisualizer::~SensorVisualizer()
{
    delete ui;
}
