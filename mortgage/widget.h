#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include "QString"
#include <QTextStream>
#include <math.h>

namespace Ui {
class Mortgage;
}

class Mortgage : public QWidget
{
    Q_OBJECT

public:
    explicit Mortgage(QWidget *parent = 0);
    ~Mortgage();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

private:
    Ui::Mortgage *ui;
    double rate = 2.5, housePrice = 200000, percentageDeposit = 15, numYears = 1, N = 1, monthleyInterestRate, borrowedAmt, monthlyRepaymentAmt;
};

#endif // WIDGET_H
