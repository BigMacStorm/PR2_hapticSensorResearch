#include "rqtdockingwindow.h"
#include "ui_rqtdockingwindow.h"

rqtDockingWindow::rqtDockingWindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::rqtDockingWindow)
{
    ui->setupUi(this);
}

rqtDockingWindow::~rqtDockingWindow()
{
    delete ui;
}
