#include "MotionCon.h"
#include "ui_MotionCon.h"
#include <QThread>
#include <sys/time.h>
#include <QDebug>

MotionCon::MotionCon(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MotionCon)
{
    ui->setupUi(this);
    server = new QTcpServer(this);
    server->listen(QHostAddress::Any,1100);
    connect(server,&QTcpServer::newConnection,[=]()
    {
        serverSocket = server->nextPendingConnection();
        connect(serverSocket,&QTcpSocket::readyRead,[=]()
        {
            QByteArray array;
            array = serverSocket->readAll();
            memcpy(&rxData,array.data(),sizeof(rxData));
            qDebug()<<rxData.iCMD<<rxData.axis;
        });
       qDebug()<<"建立新链接\n";
    });
//    this->moveToThread()
    servoCycle = new ServoCycle_t;
    servoCycle->start(QThread::NormalPriority);

}

MotionCon::~MotionCon()
{
    delete ui;
    servoCycle->exit();
}

ServoCycle_t::ServoCycle_t()
{
 //   qDebug()<<"first";
//    moveToThread(this);
 //   connect(MotionCon,,this,);
}

static struct timeval tpstart,tpend;
static double timeSpend;
void ServoCycle_t::run()
{
    while(1)
    {
        gettimeofday(&tpstart,nullptr);
        timeSpend = (tpstart.tv_sec - tpend.tv_sec)+((tpend.tv_usec-tpend.tv_usec));
  //      tpend.tv_sec = tpstart.tv_sec;
        tpend.tv_usec = tpstart.tv_usec;
 //       qDebug()<<"first!!";
        QString str = QString("%1").arg(timeSpend,0,'f',9);
   //     qDebug()<<str;
  //      qDebug()<<tpstart.tv_sec<<tpstart.tv_usec;
        msleep(20);

    }
}


void ServoCycle_t::stop()
{
    this->exit();
}
