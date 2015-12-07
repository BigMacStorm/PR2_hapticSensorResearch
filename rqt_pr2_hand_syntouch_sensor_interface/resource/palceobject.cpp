#include "palceobject.h"
#include "ui_palceobject.h"

palceObject::palceObject(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::palceObject)
{
    ui->setupUi(this);
}

palceObject::~palceObject()
{
    delete ui;
}
