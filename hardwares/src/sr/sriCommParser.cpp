#include "sriCommParser.h"



CSRICommParser::CSRICommParser()
{

}


CSRICommParser::~CSRICommParser()
{

}

bool CSRICommParser::OnReceivedData(BYTE* data, int dataLen)
{
	if (data == NULL)
	{
		return false;
	}
	if (dataLen <= 0)
	{
		return false;
	}

	return true;
}

bool CSRICommParser::OnNetworkFailure(std::string infor)
{
	return true;
}