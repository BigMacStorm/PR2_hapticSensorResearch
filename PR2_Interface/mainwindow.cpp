#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "sensorvisualizer.h"
#include "connectioninfo.h"
#include "datagraphs.h"
#include "lifetimestatistics.h"
#include "liftobject.h"
#include "placeobject.h"
#include "pr2robotvisualizer.h"
#include "programlist.h"
#include "pulsedetection.h"
#include "rqtdockingwindow.h"
#include "sensorvisualizer.h"
#include "swaphands.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(ui->IndexButton, SIGNAL(click()), this, SLOT(openNewWidget()));
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::openNewWidget()
{
}

void MainWindow::on_IndexButton_clicked()
{
    newWidget = new Widget();
    newWidget->show();
}

void MainWindow::on_connectionButton_clicked()
{
    newConnectionInfo = new ConnectionInfo();
    newConnectionInfo->show();
}

void MainWindow::on_sensorVisualizerButton_clicked()
{
    newSensorVisualizer = new SensorVisualizer();
    newSensorVisualizer->show();
}

void MainWindow::on_pr2RobotVisualizerButton_clicked()
{
    newPR2RobotVisualizer = new PR2RobotVisualizer();
    newPR2RobotVisualizer->show();
}

void MainWindow::on_lifeTimeStatsButton_clicked()
{
    newLifeTimeStatistics = new lifeTimeStatistics();
    newLifeTimeStatistics->show();
}

void MainWindow::on_dataGraphButton_clicked()
{
    newDataGraphs = new dataGraphs();
    newDataGraphs->show();
}

void MainWindow::on_programsListButton_clicked()
{
    newProgramList = new programList();
    newProgramList->show();
}

void MainWindow::on_pulseDetectionButton_clicked()
{
    newPulseDetection = new pulseDetection();
    newPulseDetection->show();
}

void MainWindow::on_liftObjectButton_clicked()
{
    newLiftObject = new liftObject();
    newLiftObject->show();
}

void MainWindow::on_placeObjectButton_clicked()
{
    newPlaceObject = new placeObject();
    newPlaceObject->show();
}

void MainWindow::on_swapHandsButton_clicked()
{
    newSwapHands = new swapHands();
    newSwapHands->show();
}

void MainWindow::on_rqtDockingWindowButton_clicked()
{
    newRqtDockingWindow = new rqtDockingWindow();
    newRqtDockingWindow->show();
}
