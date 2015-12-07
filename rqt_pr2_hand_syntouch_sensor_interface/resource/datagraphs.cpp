#include "datagraphs.h"
#include "ui_datagraphs.h"

dataGraphs::dataGraphs(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::dataGraphs)
{
    ui->setupUi(this);
}

dataGraphs::~dataGraphs()
{
    delete ui;
}
