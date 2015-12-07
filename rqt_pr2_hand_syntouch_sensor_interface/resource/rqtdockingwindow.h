#ifndef RQTDOCKINGWINDOW_H
#define RQTDOCKINGWINDOW_H

#include <QWidget>

namespace Ui {
class rqtDockingWindow;
}

class rqtDockingWindow : public QWidget
{
    Q_OBJECT

public:
    explicit rqtDockingWindow(QWidget *parent = 0);
    ~rqtDockingWindow();

private:
    Ui::rqtDockingWindow *ui;
};

#endif // RQTDOCKINGWINDOW_H
