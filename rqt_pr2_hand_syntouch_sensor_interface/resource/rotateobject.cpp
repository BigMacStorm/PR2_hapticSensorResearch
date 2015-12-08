#include "rotateobject.h"
#include "ui_rotateobject.h"

rotateObject::rotateObject(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::rotateObject)
{
    ui->setupUi(this);
}

rotateObject::~rotateObject()
{
    delete ui;
}
