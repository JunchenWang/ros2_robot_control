#include "sriCommManager.h"

CSRICommManager::CSRICommManager()
{
}

CSRICommManager::~CSRICommManager()
{
	Stop();
	
}

bool CSRICommManager::Init(const std::string &ipRemote, int portRemote, SRICommM8218CallbackFunction callback)
{
	// mTCPClient.OpenTCP(ip, 4008);
	if(mTCPClient.OpenTCP(ipRemote, portRemote))
	{
		mTCPClient.AddCommParser(&mATParser);
		mTCPClient.AddCommParser(&mM8218Parser);
		//
		SRICommNetworkFailureCallbackFunction networkFailureCallback = std::bind(&CSRICommManager::OnNetworkFailure, this, std::placeholders::_1);
		mTCPClient.SetNetworkFailureCallbackFunction(networkFailureCallback);

		SRICommATCallbackFunction atCallback = std::bind(&CSRICommManager::OnCommACK, this, std::placeholders::_1);
		mATParser.SetATCallbackFunction(atCallback);

		// SRICommM8218CallbackFunction m8218Callback = std::bind(&CSRICommManager::OnCommM8218, this,
		// 	std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
		// 	std::placeholders::_4, std::placeholders::_5, std::placeholders::_6);
		mM8218Parser.SetM8218CallbackFunction(callback);
		return true;
	}
	

	return false;
}
bool CSRICommManager::Run()
{
	if (mTCPClient.Connect() == false)
	{
		return false;
	}
	//
	if (SendCommand("SGDM", "(A01,A02,A03,A04,A05,A06);E;1;(WMA:1)") == false)
	{
		return false;
	}
	//
	if (SendCommand("DCKMD", "SUM") == false)
	{
		return false;
	}
	//
	if (SendCommand("GSD", "") == false)
	{
		// GSD has no ACK
		// return false;
	}
	return true;
}

bool CSRICommManager::Stop()
{
	mTCPClient.CloseTCP();
	return true;
}

bool CSRICommManager::OnNetworkFailure(std::string infor)
{
	printf("OnNetworkFailure = %s\n", infor.data());
	return true;
}

bool CSRICommManager::SendCommand(std::string command, std::string parames)
{
	mIsGetACK = false;
	mCommandACK = "";
	mParamesACK = "";
	std::string atCommand = "AT+" + command + "=" + parames + "\r\n";
	if (mTCPClient.OnSendData((BYTE *)atCommand.data(), (int)atCommand.length()) == false)
	{
		return false;
	}
	//
#ifdef IS_WINDOWS_OS
	std::clock_t start = clock();
	while (true)
	{
		if (mIsGetACK == true)
		{
			break;
		}
		std::clock_t end = clock();
		long span = end - start;
		if (span >= 10000) // 10s
		{
			return false;
		}
	}
#else
	timeval start, end;
	gettimeofday(&start, NULL);
	while (true)
	{
		if (mIsGetACK == true)
		{
			break;
		}
		gettimeofday(&end, NULL);
		long span = 1000 * (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000;
		if (span >= 10000) // 10s
		{
			return false;
		}
	}
#endif
	//
	if (mCommandACK != command)
	{
		return false;
	}
	//
	printf("ACK+%s=%s", mCommandACK.data(), mParamesACK.data());
	//
	return true;
}
bool CSRICommManager::OnCommACK(std::string command)
{
	int index = (int)command.find('=');
	if (index == -1)
	{
		mCommandACK = command;
		mParamesACK = "";
	}
	else
	{
		mCommandACK = command.substr(0, index);
		mParamesACK = command.substr(index + 1);
	}
	mCommandACK = mCommandACK.substr(4);

	mIsGetACK = true;

	return true;
}

bool CSRICommManager::OnCommM8218(float fx, float fy, float fz, float mx, float my, float mz)
{
	printf("M8218 = %f, %f, %f,   %f, %f, %f\n", fx, fy, fz, mx, my, mz);

	return true;
}