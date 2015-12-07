#include "pulsedetection.h"
#include "ui_pulsedetection.h"

pulseDetection::pulseDetection(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::pulseDetection)
{
    ui->setupUi(this);
}

pulseDetection::~pulseDetection()
{
    delete ui;
}
