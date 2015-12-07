#include "placeobject.h"
#include "ui_placeobject.h"

placeObject::placeObject(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::placeObject)
{
    ui->setupUi(this);
}

placeObject::~placeObject()
{
    delete ui;
}
