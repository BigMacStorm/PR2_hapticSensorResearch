#include "swaphands.h"
#include "ui_swaphands.h"

swapHands::swapHands(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::swapHands)
{
    ui->setupUi(this);
}

swapHands::~swapHands()
{
    delete ui;
}
