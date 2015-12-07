#include "liftobject.h"
#include "ui_liftobject.h"

liftObject::liftObject(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::liftObject)
{
    ui->setupUi(this);
}

liftObject::~liftObject()
{
    delete ui;
}
