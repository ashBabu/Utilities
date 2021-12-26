#include "widget.h"
#include <QApplication>
#include <QTextStream>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

//    QTextStream qin(stdin);
//    QTextStream qout(stdout);

//    qout << "Enter the annual rate of interest: \n";
//    bool ok1;
//    float r = qin.readLine().toFloat(&ok1);
    Mortgage w;
//    w.setWindowTitle(QDir::currentPath());

    w.show();

    return a.exec();
}
