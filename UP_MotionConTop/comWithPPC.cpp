#include "comWithPPC.h"

class UPMotionCon;
client_t::client_t()
{
}
QString client_t::getSocketError()
{
    return clientSock->errorString();
}
void client_t::socketClose()
{
    clientSock->close();
}

RXData_t::RXData_t()
{

}

