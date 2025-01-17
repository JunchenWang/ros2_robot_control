#include "sriCommATParser.h"



CSRICommATParser::CSRICommATParser()
{
	mCircularBuffer.Init(102400);//100KB
	mAtCallbackFunction = NULL;
}


CSRICommATParser::~CSRICommATParser()
{
}
bool CSRICommATParser::SetATCallbackFunction(SRICommATCallbackFunction atCallbackFunction)
{
	mAtCallbackFunction = atCallbackFunction;
	return false;
}
bool CSRICommATParser::OnReceivedData(BYTE* data, int dataLen)
{
	if (data == NULL)
	{
		return false;
	}
	if (dataLen <= 0)
	{
		return false;
	}
	mCircularBuffer.Write(data, dataLen);
	//
	int delLen = 0;
	std::string ack = "";
	if (ParseDataFromBuffer(delLen, ack) == false)
	{
		mCircularBuffer.Clear(delLen);
		return false;
	}
	mCircularBuffer.Clear(delLen);
	
	if (mAtCallbackFunction != NULL)
	{
		mAtCallbackFunction(ack);
	}
	return true;
}

bool CSRICommATParser::OnNetworkFailure(std::string infor)
{
	return true;
}

bool CSRICommATParser::ParseDataFromBuffer(int& delLen, std::string& ack)
{
	int dataLen = 0;
	BYTE* data = mCircularBuffer.ReadTry(dataLen);
	if (data == NULL)
	{
		return false;
	}
	// ACK +
	if (dataLen < 4)
	{
		delLen = 0;
		ack = "";
		return false;
	}
	//
	int headIndex = ParseGetHeadIndex(data, dataLen);
	if (headIndex == -1)
	{
		delLen = dataLen - 3; //找不到帧头，删除当前所有数据，保留1/4个帧头
		ack = "";
		return false;
	}
	//
	int endIndex = ParseGetEndIndex(data, dataLen, headIndex + 4);
	if (endIndex == -1)
	{
		delLen = headIndex;
		ack = "";
		return false;
	}
	//
	int len = endIndex - headIndex + 2;
	char* command = new char[len];
	memcpy(command, data + headIndex, len);
	std::string commandStr = command;
	delete command;
	ack = commandStr;
	delLen = endIndex + 2;
	return true;
}
int CSRICommATParser::ParseGetHeadIndex(BYTE* data, int dataLen)
{
	int headIndex = -1;
	for (int i = 0; i < dataLen - 3; i++)
	{
		// ACK+
		if ((data[i] == 65) && (data[i + 1] == 67) && (data[i + 2] == 75) && (data[i + 3] == 43))
		{
			headIndex = i;
			break;
		}
	}
	return headIndex;
}
int CSRICommATParser::ParseGetEndIndex(BYTE* data, int dataLen, int index)
{
	int endIndex = -1;
	for (int i = index; i < dataLen - 1; i++)
	{
		if ((data[i] == 13) && (data[i + 1] == 10))
		{
			endIndex = i;
			break;
		}
	}
	return endIndex;
}