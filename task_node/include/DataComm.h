#ifndef DATACOMM_H
#define DATACOMM_H

#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <thread>
struct RobotData
{
    double t;
    float q[4][7];
};

typedef void (*ReceiveCallBack) (void *, const RobotData &);



class DataComm
{
private:
    DataComm();
    ~DataComm();

public:
    static DataComm *getInstance();
    void setDestAddress(const char *ip, int port);
    int sendLine(const char * msg);
    int sendMsg(const char * msg);
    int sendRobotStatus(const RobotData & s);
    void startReceiveService(ReceiveCallBack fnc, void *pClientData, int port, double dt = 0.0);
    void stopReceiveService();
private:
    void setBufferSize(int n);
    int socket_handle;
    struct sockaddr_in addr;
    char *buffer;
    int sz;
    pthread_t service;
};

template <class T>
void log2Channel(RobotData &roboData, int c, const T *data, int n, int offset = 0)
{
    for (int i = 0; i < n; i++)
        roboData.q[c][offset + i] = static_cast<float>(data[i]);
}
#endif
