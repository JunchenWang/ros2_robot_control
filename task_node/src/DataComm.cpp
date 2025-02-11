#include "DataComm.h"
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <chrono>
#include <unistd.h>
#include <fstream>
#include <vector>
using namespace std::chrono;

using namespace std;

static volatile bool loop = true;
struct ClientData
{ 
    ReceiveCallBack fnc;
    int port;
    double dt;
    void *pClientData;
};

void log2file(std::ofstream &fout, std::vector<RobotData> &frames)
{
    for (int k = 0; k < frames.size(); k++)
    {
        fout << frames[k].t << "\n";
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 7; j++)
                fout << frames[k].q[i][j] << (j == 6 ? "\n" : " ");

        fout << "\n";
    }
}

void *service_func(void *pdata)
{
    int port = reinterpret_cast<ClientData *>(pdata)->port;
    ReceiveCallBack func = reinterpret_cast<ClientData *>(pdata)->fnc;
    double dt = reinterpret_cast<ClientData *>(pdata)->dt;
    void *pClientData = reinterpret_cast<ClientData *>(pdata)->pClientData;
    int socket_handle = socket(AF_INET, SOCK_DGRAM, 0);

    if (socket_handle == -1)
    {
        printf("create socket in service failed\n");
        return nullptr;
    }
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(socket_handle, (struct sockaddr *)&addr, sizeof(addr)))
    {
        printf("bind socket in service failed\n");
        return nullptr;
    }
    ofstream fout("log.txt");
    fout.setf(std::ios::fixed);
    fout.precision(9);
    RobotData data;
    vector<RobotData> frames;
    struct timeval read_timeout;
    read_timeout.tv_sec = 0;
    read_timeout.tv_usec = 10;
    // non-blocking
    if (setsockopt(socket_handle, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout)))
    {
        printf("set socket time out in service failed\n");
        return nullptr;
    }
    while (loop)
    {
        auto t_start = high_resolution_clock::now();
        int resv_num = recvfrom(socket_handle, &data, sizeof(data), 0, nullptr, nullptr);
        if (resv_num > 0)
        {
            if (func)
                func(pClientData, data);
            // printf("received: %f\n", data.q[0]);
            frames.push_back(data);
        }
        if (dt > 0)
        {
            auto t_stop = high_resolution_clock::now();
            auto t_duration = std::chrono::duration<double>(t_stop - t_start);
            if (t_duration.count() < dt)
                std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
        }
    }
    log2file(fout, frames);
    close(socket_handle);

    printf("stop receiving\n");
    return nullptr;
}

DataComm::DataComm()
{
    socket_handle = socket(AF_INET, SOCK_DGRAM, 0); /* Handle to UDP socket used to communicate with Net F/T. */
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    buffer = new char[2048];
    sz = 2048;
    service = 0;
}

DataComm::~DataComm()
{
    close(socket_handle);
    stopReceiveService();
}

DataComm *DataComm::getInstance()
{
    static DataComm comm;
    return &comm;
}

void DataComm::setDestAddress(const char *ip, int port)
{
    addr.sin_port = htons((short)port);
    addr.sin_addr.s_addr = inet_addr(ip);
}

int DataComm::sendLine(const char *msg)
{
    int n = strlen(msg);
    setBufferSize(n + 1);
    memcpy(buffer, msg, n);
    buffer[n] = '\n';
    // buffer[n + 1] = 0;
    return sendto(socket_handle, buffer, n + 1, 0, (struct sockaddr *)&addr, sizeof(addr));
}

int DataComm::sendMsg(const char *msg)
{
    int n = strlen(msg);
    return sendto(socket_handle, msg, n, 0, (struct sockaddr *)&addr, sizeof(addr));
}

int DataComm::sendRobotStatus(const RobotData &s)
{
    return sendto(socket_handle, &s, sizeof(s), 0, (struct sockaddr *)&addr, sizeof(addr));
}

void DataComm::startReceiveService(ReceiveCallBack fnc, void *pClientData, int port, double dt)
{
    static ClientData data;
    if (service)
    {
        printf("there is already a receive thread\n");
        return;
    }
    data.fnc = fnc;
    data.port = port;
    data.dt = dt;
    data.pClientData = pClientData;
    int ret = pthread_create(&service, nullptr, service_func, &data);
    if (ret)
    {
        printf("create data communication thread failed\n");
    }
}

void DataComm::stopReceiveService()
{
    if (service)
    {
        loop = false;
        pthread_join(service, nullptr);
        service = 0;
    }
}

void DataComm::setBufferSize(int n)
{
    if (n > sz)
    {
        delete[] buffer;
        buffer = new char[n];
        sz = 2 * n;
    }
}
