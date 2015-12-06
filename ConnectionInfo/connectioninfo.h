#ifndef CONNECTIONINFO_H
#define CONNECTIONINFO_H

#include <QWidget>

namespace Ui {
class ConnectionInfo;
}

class ConnectionInfo : public QWidget
{
    Q_OBJECT

public:
    explicit ConnectionInfo(QWidget *parent = 0);
    ~ConnectionInfo();

private:
    Ui::ConnectionInfo *ui;
};

#endif // CONNECTIONINFO_H
