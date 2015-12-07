#include "swaphands.h"
#include "ui_swaphands.h"

swapHands::swapHands(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::swapHands)
{
    ui->setupUi(this);
}

swapHands::~swapHands()
{
    delete ui;
}
