#ifndef RQTDOCKINGWINDOW_H
#define RQTDOCKINGWINDOW_H

#include <QDialog>

namespace Ui {
class rqtDockingWindow;
}

class rqtDockingWindow : public QDialog
{
    Q_OBJECT

public:
    explicit rqtDockingWindow(QWidget *parent = 0);
    ~rqtDockingWindow();

private:
    Ui::rqtDockingWindow *ui;
};

#endif // RQTDOCKINGWINDOW_H
