#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "widget.h"
#include "connectioninfo.h"
#include "sensorvisualizer.h"
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

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void openNewWidget();
    void on_IndexButton_clicked();
    void on_connectionButton_clicked();
    void on_sensorVisualizerButton_clicked();
    void on_pr2RobotVisualizerButton_clicked();
    void on_lifeTimeStatsButton_clicked();
    void on_dataGraphButton_clicked();
    void on_programsListButton_clicked();
    void on_pulseDetectionButton_clicked();
    void on_liftObjectButton_clicked();
    void on_placeObjectButton_clicked();
    void on_swapHandsButton_clicked();
    void on_rqtDockingWindowButton_clicked();

private:
    Ui::MainWindow *ui;
    Widget *newWidget;
    ConnectionInfo *newConnectionInfo;
    SensorVisualizer *newSensorVisualizer;
    dataGraphs *newDataGraphs;
    lifeTimeStatistics *newLifeTimeStatistics;
    liftObject *newLiftObject;
    placeObject *newPlaceObject;
    PR2RobotVisualizer *newPR2RobotVisualizer;
    programList *newProgramList;
    pulseDetection *newPulseDetection;
    rqtDockingWindow *newRqtDockingWindow;
    swapHands *newSwapHands;
};

#endif // MAINWINDOW_H
