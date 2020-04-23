#ifndef MOTIONCON_H
#define MOTIONCON_H

#include <QMainWindow>
#include <QThread>
#include <QTcpServer>
#include <QTcpSocket>
#include "StateMachine.h"
#include "ComWithWin.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MotionCon; }
QT_END_NAMESPACE

class ServoCycle_t;

class MotionCon : public QMainWindow
{
    Q_OBJECT

public:
    MotionCon(QWidget *parent = nullptr);
    ~MotionCon();
    RXData_t rxData;

private:
    Ui::MotionCon *ui;
    QTcpServer *server;
    QTcpSocket *serverSocket;
    ServoCycle_t *servoCycle;
//    RXData_t rxData;
};

class ServoCycle_t:public QThread    //public QObject
{
    Q_OBJECT
public:
    ServoCycle_t();
    void run();
//public slots:
    void ServoCycle();
    void stop();
};

#endif // MOTIONCON_H
