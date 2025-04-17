#include "sriCommM8218Parser.h"



CSRICommM8218Parser::CSRICommM8218Parser()
{
	mCircularBuffer.Init(102400);//100KB
	mM8218CallbackFunction = NULL;
}


CSRICommM8218Parser::~CSRICommM8218Parser()
{
}

bool CSRICommM8218Parser::SetM8218CallbackFunction(SRICommM8218CallbackFunction m8218CallbackFunction)
{
	mM8218CallbackFunction = m8218CallbackFunction;
	return true;
}
bool CSRICommM8218Parser::OnReceivedData(BYTE* data, int dataLen)
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
	float fx;
	float fy;
	float fz;
	float mx;
	float my;
	float mz;
	if (ParseDataFromBuffer(delLen, fx, fy, fz, mx, my, mz) == false)
	{
		mCircularBuffer.Clear(delLen);
		return false;
	}	
	mCircularBuffer.Clear(delLen);	
	//
	if (mM8218CallbackFunction != NULL)
	{
		mM8218CallbackFunction(fx, fy, fz, mx, my, mz);
	}
	return true;
}

bool CSRICommM8218Parser::OnNetworkFailure(std::string &/*infor*/)
{
	return true;
}


bool CSRICommM8218Parser::ParseDataFromBuffer(int& delLen, float& fx, float& fy, float& fz, float& mx, float& my, float& mz)
{
	delLen = 0;
	int dataLen = 0;
	BYTE* data = mCircularBuffer.ReadTry(dataLen);
	if (data == NULL)
	{
		return false;
	}
	//
	int headIndex = ParseGetHeadIndex(data, dataLen);
	if (headIndex == -1)
	{
		delLen = dataLen - 1; 
		return false;
	}
	//
	if (headIndex + 31 > dataLen)
	{
		//
		delLen = headIndex; 
		return false;
	}
	//
	int index = headIndex + 2;
	int frameLen = data[index + 0] * 256 + data[index + 1];
	if (frameLen != 27)
	{
		delLen = index; 
		return false;
	}
	index = index + 2;
	//
	//int frameNo = data[index + 0] * 256 + data[index + 1];
	index = index + 2;
	//
	fx = 0;
	memcpy(&fx, data + index, 4);
	index = index + 4;
	fy = 0;
	memcpy(&fy, data + index, 4);
	index = index + 4;
	fz = 0;
	memcpy(&fz, data + index, 4);
	index = index + 4;

	mx = 0;
	memcpy(&mx, data + index, 4);
	index = index + 4;
	my = 0;
	memcpy(&my, data + index, 4);
	index = index + 4;
	mz = 0;
	memcpy(&mz, data + index, 4);
	index = index + 4;

	//
	BYTE check = data[index];
	index = index + 1;
	//
	delLen = index; 
	//
	BYTE checkNew = 0x00;
	for (int i = headIndex + 6; i <= headIndex + 4 + 24 +1; i++)
	{
		checkNew = (checkNew + data[i]) ;
	}
	if (checkNew != check)
	{
		return false;
	}


	return true;
}
int CSRICommM8218Parser::ParseGetHeadIndex(BYTE* data, int dataLen)
{
	int headIndex = -1;
	for (int i = 0; i < dataLen - 1; i++)
	{
		if ((data[i] == 0xAA) && (data[i + 1] == 0x55))
		{
			headIndex = i;
		}
	}
	return headIndex;
}