#ifndef SRI_COMM_AT_PARSER_H
#define SRI_COMM_AT_PARSER_H

#include "sriCommParser.h"
#include "sriCommCircularBuffer.h"

class CSRICommATParser : public CSRICommParser
{
public:
	CSRICommATParser();
	~CSRICommATParser();

	bool SetATCallbackFunction(SRICommATCallbackFunction atCallbackFunction);

	bool OnReceivedData(BYTE* data, int dataLen);
	bool OnNetworkFailure(std::string infor);

private:
	CSRICommCircularBuffer mCircularBuffer;

	SRICommATCallbackFunction mAtCallbackFunction;

	bool ParseDataFromBuffer(int& delLen, std::string& ack);
	int ParseGetHeadIndex(BYTE* data, int dataLen);
	int ParseGetEndIndex(BYTE* data, int dataLen, int index);

};

#endif
