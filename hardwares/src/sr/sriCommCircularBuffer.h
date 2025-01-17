#ifndef SRI_COMM_CIRCULAR_BUFFER_H
#define SRI_COMM_CIRCULAR_BUFFER_H

#include "sriCommDefine.h"
class CSRICommCircularBuffer
{
public:
	CSRICommCircularBuffer();
	~CSRICommCircularBuffer();

	bool Init(int bufferMaxSize=10240);
	int GetLength();
	int GetLength(int& wIndex, int& rIndex);
	bool Clear();
	bool Clear(int clearLen);

	int Write(BYTE* data, int dataLen);
	int Write(BYTE data);

	BYTE* Read(int& dataLen, int readLen = 0, bool delData = true);
	BYTE* ReadTry(int& dataLen, int readLen = 0);

private:
	BYTE* mBuffer;
	int mBufferSize;
	int mWIndex;
	int mRIndex;
	int mTotalWCount;
	int mTotalRCount;
};

#endif

