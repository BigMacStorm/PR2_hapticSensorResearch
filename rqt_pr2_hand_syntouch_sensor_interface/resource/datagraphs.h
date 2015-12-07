#ifndef DATAGRAPHS_H
#define DATAGRAPHS_H

#include <QWidget>

namespace Ui {
class dataGraphs;
}

class dataGraphs : public QWidget
{
    Q_OBJECT

public:
    explicit dataGraphs(QWidget *parent = 0);
    ~dataGraphs();

private:
    Ui::dataGraphs *ui;
};

#endif // DATAGRAPHS_H
