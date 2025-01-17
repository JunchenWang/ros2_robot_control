#ifndef SRI_COMM_TCP_CLIENT_H
#define SRI_COMM_TCP_CLIENT_H

#include "sriCommDefine.h"
#include "sriCommParser.h"

class CSRICommTCPClient
{
public:
	CSRICommTCPClient();
	~CSRICommTCPClient();

	bool OpenTCP(const std::string &ipRemote, int portRemote, const std::string &ipLocal = "", int portLocal = 0);
	bool CloseTCP();

	bool Connect();
	bool Disconnect();
	bool ReConnect();

	bool OnReceivedData(BYTE* data, int dataLen);
	bool OnSendData(BYTE* data, int dataLen);

	bool AddCommParser(CSRICommParser* parser);

	bool SetNetworkFailureCallbackFunction(SRICommNetworkFailureCallbackFunction networkFailureCallback);

	std::string GetLastError();
private:
	std::string mIpRemote;
	int mPortRemote;
	std::string mIpLocal;
	int mPortLocal;


	int mSocket;
	struct sockaddr_in mLocalAddr;
	struct sockaddr_in mRemoteAddr;
	bool ConnectTCP();
	bool BindLocalIP();
	bool SetKeepAlive();

	std::thread mThread;
	bool mIsStopThread;
	bool mIsTreadStoped;
	bool OpenThread();
	bool CloseThread();
	void TCPClientReceiverThread(int code);

	
	bool CheckTimeoutError();
	bool OnNetworkFailure();

	std::vector<CSRICommParser*> mParserList;

	SRICommNetworkFailureCallbackFunction mNetworkFailureCallback;

	std::string mLastError;
	void GetLastSocketError(std::string functionName);
};

#endif

