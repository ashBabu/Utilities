#include "widget.h"
#include "ui_widget.h"
#include <stdlib.h>


Mortgage::Mortgage(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Mortgage)
{
    this->setWindowTitle("Mortgage");
    ui->setupUi(this);
}

Mortgage::~Mortgage()
{
    delete ui;
}

void Mortgage::on_pushButton_clicked()
{
    //    https://en.wikipedia.org/wiki/Mortgage_calculator

    rate = ui->rate->value();
    housePrice = ui->housePrice->value();
    percentageDeposit = ui->percentDeposit->value();
    numYears = ui->numYears->value();

    float deposit = housePrice * percentageDeposit/100;

    monthleyInterestRate = rate / 12 / 100;
    borrowedAmt = housePrice - deposit;
    int totNumMonthlyPayments = numYears * 12;

    float temp1 = pow((1 + monthleyInterestRate), totNumMonthlyPayments);
    monthlyRepaymentAmt = monthleyInterestRate * borrowedAmt * temp1 / (temp1 - 1);

    ui->depositNeeded->setText("The deposit amount is: " + QString::number(deposit, 'g', 10));
    ui->borrowedAmount->setText("The borrowed amount is: " + QString::number(borrowedAmt, 'g', 10));
    ui->monthlyRepayment->setText("The monthly repayment is: " + QString::number(monthlyRepaymentAmt, 'g', 10));


}

void Mortgage::on_pushButton_2_clicked()
{
    // To calculate the remaining borrowed amount after N years


    N = ui->N->value();
    float NinMonths = N * 12;
    float temp2 = pow((1 + monthleyInterestRate), NinMonths);
    float OwedAmountAfter_N_months = (temp2 * borrowedAmt) - ((temp2 - 1) * monthlyRepaymentAmt/monthleyInterestRate);

    ui->remBorrowedAmt->setText("The remaining borrowed amount is: " + QString::number(OwedAmountAfter_N_months, 'g', 10));
}
