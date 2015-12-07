#include "connectioninfo.h"
#include "ui_connectioninfo.h"

ConnectionInfo::ConnectionInfo(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ConnectionInfo)
{
    ui->setupUi(this);
}

ConnectionInfo::~ConnectionInfo()
{
    delete ui;
}
