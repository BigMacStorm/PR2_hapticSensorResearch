#include "lifetimestatistics.h"
#include "ui_lifetimestatistics.h"

lifeTimeStatistics::lifeTimeStatistics(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::lifeTimeStatistics)
{
    ui->setupUi(this);
}

lifeTimeStatistics::~lifeTimeStatistics()
{
    delete ui;
}
