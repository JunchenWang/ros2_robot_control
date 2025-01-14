#include "sriCommTCPClient.h"



CSRICommTCPClient::CSRICommTCPClient()
{
	mIpRemote = "";
	mPortRemote = 0;
	mIpLocal = "";
	mPortLocal = 0;

	mSocket = -1;

	mNetworkFailureCallback = NULL;
	mLastError = "";

#ifdef  IS_WINDOWS_OS
	WSADATA wsaData;
	WORD  wVersionRequested = MAKEWORD(2, 2);
	WSAStartup(wVersionRequested, &wsaData);
#endif
}


CSRICommTCPClient::~CSRICommTCPClient()
{
	
	mParserList.clear();
	CloseThread();
	mThread.join();	
}

bool CSRICommTCPClient::OpenTCP(const std::string &ipRemote, int portRemote, const std::string &ipLocal, int portLocal)
{	
	mIpRemote = ipRemote;
	mPortRemote = portRemote;
	mIpLocal = ipLocal;
	mPortLocal = portLocal;
	try
	{
		CloseTCP();
		//
		mSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (-1 == mSocket)
		{
			GetLastSocketError("socket()");
			return false;
		}
		//
		if (BindLocalIP() == false)
		{
			return false;
		}
		//
		if (SetKeepAlive() == false)
		{
			return false;
		}
		//
		memset(&mRemoteAddr, 0, sizeof(mRemoteAddr));
		mRemoteAddr.sin_family = AF_INET;
		mRemoteAddr.sin_port = htons(mPortRemote);
		if (inet_pton(AF_INET, mIpRemote.data(), &mRemoteAddr.sin_addr) <= 0)
		{
			GetLastSocketError("inet_pton()");
			return false;
		}
		


	}
	catch(std::exception ex)
	{
		mLastError = ex.what();
		return false;
	}
	return true;
}

bool CSRICommTCPClient::CloseTCP()
{
	CloseThread();

	if (mSocket != -1)
	{
#ifdef  IS_WINDOWS_OS
		closesocket(mSocket);
#else
		close(mSocket);
#endif
		mSocket = -1;
	}
	
	//
	return true;
}

bool CSRICommTCPClient::Connect()
{
	if (ConnectTCP() == false)
	{
		return false;
	}
	//
	if (OpenThread() == false)
	{
		GetLastSocketError("OpenThread()");
		return false;
	}

	return true;
}

bool CSRICommTCPClient::Disconnect()
{
	//shutdown(mSocket, 2);
	return true;
}

bool CSRICommTCPClient::ReConnect()
{
	Disconnect();
	return ConnectTCP();
}

bool CSRICommTCPClient::ConnectTCP()
{
	//
	Disconnect();
	//
#ifdef  IS_WINDOWS_OS
	unsigned long blockFlag = 1;
	if (ioctlsocket(mSocket, FIONBIO, (unsigned long*)&blockFlag) < 0)
	{
		GetLastSocketError("ioctlsocket() FIONBIO");
		return false;
	}
#else
	int blockFlag = 0;
	if (ioctl(mSocket, FIONBIO, (char*)(&blockFlag)) < 0)
	{
		GetLastSocketError("ioctlsocket() FIONBIO");
		return false;
	}
#endif
	//
	bool isConnected = false;
	if (connect(mSocket, (struct sockaddr*) &mRemoteAddr, sizeof(mRemoteAddr)) == -1)
	{
		struct timeval timeout = { 0 };
		timeout.tv_sec = 1;
		timeout.tv_usec = 0;

		fd_set fdwrite;
		FD_ZERO(&fdwrite);
		FD_SET(mSocket, &fdwrite);

		int error = -1;
		int len = sizeof(int);
		if (select(mSocket, 0, &fdwrite, 0, &timeout) > 0)
		{
			getsockopt(mSocket, SOL_SOCKET, SO_ERROR, (char*)&error, (socklen_t*)&len);
			if (error != 0)
			{
				isConnected = false;
			}
			isConnected = true;
		}
		else
		{
			isConnected = false;
		}
	}
	else
	{
		isConnected = true;
	}
	//
#ifdef  IS_WINDOWS_OS
	blockFlag = 1;
	if (ioctlsocket(mSocket, FIONBIO, (unsigned long*)&blockFlag) < 0)
	{
		GetLastSocketError("ioctlsocket() FIONBIO");
		return false;
	}
#else
	blockFlag = 0;
	if (ioctl(mSocket, FIONBIO,  (char*)(&blockFlag)) < 0)
	{
		GetLastSocketError("ioctlsocket() FIONBIO");
		return false;
	}
#endif
	//
	if (isConnected == false)
	{
		GetLastSocketError("ConnectTCP() isConnected == false");
		return false;
	}
	return true;
}

bool CSRICommTCPClient::BindLocalIP()
{
	if (("" != mIpLocal) && (0 != mPortLocal))
	{
		memset(&mLocalAddr, 0, sizeof(mLocalAddr));
		mLocalAddr.sin_family = AF_INET;
		mLocalAddr.sin_port = htons(mPortLocal);
		if (inet_pton(AF_INET, mIpLocal.data(), &mLocalAddr.sin_addr) <= 0)
		{
			GetLastSocketError("inet_pton()");
			return false;
		}
		if (bind(mSocket, (struct sockaddr*)&mLocalAddr, sizeof(mLocalAddr)) == -1)
		{
			GetLastSocketError("bind()");
			return false;
		}
	}
	return true;
}

bool CSRICommTCPClient::SetKeepAlive()
{
	//
	int keepAlive = 1;		//
	int keepIdle = 1;		// 
	int keepInterval = 10;	//
	int keepCount = 3;		//			
	int netTimeout = 10000;	//
#ifdef  IS_WINDOWS_OS
	if (setsockopt(mSocket, SOL_SOCKET, SO_KEEPALIVE, (const char *)&keepAlive, sizeof(keepAlive)) < 0)
	{
		GetLastSocketError("setsockopt() SO_KEEPALIVE");
		return false;
	}

	if (setsockopt(mSocket, IPPROTO_TCP, TCP_KEEPIDLE, (const char *)&keepIdle, sizeof(keepIdle)) < 0) 
	{
		GetLastSocketError("setsockopt() TCP_KEEPIDLE");
		return false;
	}

	if (setsockopt(mSocket, IPPROTO_TCP, TCP_KEEPINTVL, (const char *)&keepInterval, sizeof(keepInterval)) < 0)
	{
		GetLastSocketError("setsockopt() TCP_KEEPINTVL");
		return false;
	}
	if (setsockopt(mSocket, IPPROTO_TCP, TCP_KEEPCNT, (const char *)&keepCount, sizeof(keepCount)) < 0)
	{
		GetLastSocketError("setsockopt() TCP_KEEPCNT");
		return false;
	}
									
	if (setsockopt(mSocket, SOL_SOCKET, SO_SNDTIMEO, (const char*)&netTimeout, sizeof(int)) < 0)
	{
		GetLastSocketError("setsockopt() SO_SNDTIMEO");
	}
	return true;
#else
	if (setsockopt(mSocket, SOL_SOCKET, SO_KEEPALIVE, (void *)&keepAlive, sizeof(keepAlive)) < 0)
	{
		GetLastSocketError("setsockopt() SO_KEEPALIVE");
		return false;
	}

	if (setsockopt(mSocket, IPPROTO_TCP, TCP_KEEPIDLE, (void *)&keepIdle, sizeof(keepIdle)) < 0)
	{
		GetLastSocketError("setsockopt() TCP_KEEPIDLE");
		return false;
	}

	if (setsockopt(mSocket, IPPROTO_TCP, TCP_KEEPINTVL, (void *)&keepInterval, sizeof(keepInterval)) < 0) 
	{
		GetLastSocketError("setsockopt() TCP_KEEPINTVL");
		return false;
	}
	if (setsockopt(mSocket, IPPROTO_TCP, TCP_KEEPCNT, (void *)&keepCount, sizeof(keepCount)) < 0)
	{
		GetLastSocketError("setsockopt() TCP_KEEPCNT");
		return false;
	}
	struct timeval timeout = { 0,0 };//s
	timeout.tv_sec = netTimeout;
	if (setsockopt(mSocket, SOL_SOCKET, SO_SNDTIMEO, (void*)&timeout, sizeof(timeout)) < 0)
	{
		GetLastSocketError("setsockopt() SO_SNDTIMEO");
	}	
	return true;
#endif
}

bool CSRICommTCPClient::OpenThread()
{
	mThread = std::thread(&CSRICommTCPClient::TCPClientReceiverThread, this, 0);
	return true;
}
bool CSRICommTCPClient::CloseThread()
{
	mIsStopThread = true;
	std::clock_t start = clock();
	while (true)
	{
		if (mIsTreadStoped == true)
		{
			break;
		}
		std::clock_t end = clock();
		double span = end - start;
		if (span >= 1)//1s
		{
			return false;
		}
	}
	return true;
}
void CSRICommTCPClient::TCPClientReceiverThread(int code)
{
	mIsStopThread = false;
	mIsTreadStoped = false;

	int dataBufferLen = 8192;//8K
	BYTE* dataBuffer = new BYTE[dataBufferLen];
	memset(dataBuffer, 0, dataBufferLen);
	int recvDataLen = 0;

	fd_set readfds;
	fd_set writefds;
	fd_set exceptfds;
	memset(&readfds, 0, sizeof(fd_set));
	memset(&writefds, 0, sizeof(fd_set));
	memset(&exceptfds, 0, sizeof(fd_set));
	timeval timeout;
	memset(&timeout, 0, sizeof(timeval));
	timeout.tv_sec = 1;

	FD_SET(mSocket, &readfds);
	FD_SET(mSocket, &writefds);
	FD_SET(mSocket, &exceptfds);
	while (true)
	{
		if (mIsStopThread == true)
		{
			break;
		}

		recvDataLen = recv(mSocket, (char*)dataBuffer, dataBufferLen, 0);
		if ((recvDataLen > 0) && (recvDataLen <= dataBufferLen))
		{
			//
			OnReceivedData(dataBuffer, recvDataLen);
			memset(dataBuffer, 0, recvDataLen);
		}
		else if (recvDataLen == 0)
		{
			continue;
		}
		else
		{
			if (CheckTimeoutError() == true)
			{
				GetLastSocketError("recv() Timeout"); 
				break;
			}
			
		}

	}
	delete[] dataBuffer;
	mIsTreadStoped = true;
}

bool CSRICommTCPClient::OnReceivedData(BYTE* data, int dataLen)
{
	for (size_t i = 0; i < mParserList.size(); ++i)
	{
		CSRICommParser* parser = mParserList[i];
		if (parser != NULL)
		{
			parser->OnReceivedData(data, dataLen);
		}
	}
	return true;
}
bool CSRICommTCPClient::CheckTimeoutError()
{
#ifdef  IS_WINDOWS_OS
	int errorCode = WSAGetLastError();
	if (WSAETIMEDOUT == errorCode)//
	{
		return true;
	}
#else
	//#define ETIMEDOUT       110     /* Connection timed out */  
	if (ETIMEDOUT == errno)
	{
		return true;
	}
#endif
	return false;
}
bool CSRICommTCPClient::OnNetworkFailure()
{
	for (size_t i = 0; i < mParserList.size(); ++i)
	{
		CSRICommParser* parser = mParserList[i];
		if (parser != NULL)
		{
			parser->OnNetworkFailure(mLastError);
		}
	}
	if (mNetworkFailureCallback != NULL)
	{
		mNetworkFailureCallback(mLastError);
	}
	return true;
}


bool CSRICommTCPClient::OnSendData(BYTE* data, int dataLen)
{
	if (data == NULL)
	{
		return false;
	}
	if (dataLen <= 0)
	{
		return false;
	}
	try
	{
		int sendRet = send(mSocket, (char*)data, dataLen, 0);
		if (sendRet != dataLen)
		{
			GetLastSocketError("send()");			
			return false;
		}
	}
	catch (std::exception ex)
	{
		mLastError = ex.what();
		OnNetworkFailure();
		return false;
	}
	return true;
}

bool CSRICommTCPClient::AddCommParser(CSRICommParser* parser)
{
	mParserList.push_back(parser);
	return true;
}

bool CSRICommTCPClient::SetNetworkFailureCallbackFunction(SRICommNetworkFailureCallbackFunction networkFailureCallback)
{
	mNetworkFailureCallback = networkFailureCallback;
	return true;
}

std::string CSRICommTCPClient::GetLastError()
{
	return mLastError;
}

void CSRICommTCPClient::GetLastSocketError(std::string functionName = "")
{
	mLastError = "";
	#ifdef  IS_WINDOWS_OS
		char buffer[2048];
		memset(buffer, 0, 2048);
		sprintf(buffer, "Socket %s error: %d\n", functionName.data(), WSAGetLastError());
		mLastError = buffer;
	#else
		char buffer[2048];
		memset(buffer, 0, 2048);
		sprintf(buffer, "Socket %s error: %s(errno: %d)\n", functionName.data(), strerror(errno), errno);
		mLastError = buffer;
	#endif

	OnNetworkFailure();
}