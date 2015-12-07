#include "programlist.h"
#include "ui_programlist.h"

programList::programList(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::programList)
{
    ui->setupUi(this);
}

programList::~programList()
{
    delete ui;
}
