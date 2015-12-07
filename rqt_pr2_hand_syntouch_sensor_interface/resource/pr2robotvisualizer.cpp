#include "pr2robotvisualizer.h"
#include "ui_pr2robotvisualizer.h"

PR2RobotVisualizer::PR2RobotVisualizer(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PR2RobotVisualizer)
{
    ui->setupUi(this);
}

PR2RobotVisualizer::~PR2RobotVisualizer()
{
    delete ui;
}
