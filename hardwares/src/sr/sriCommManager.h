#ifndef SRI_COMM_MANAGER_H
#define SRI_COMM_MANAGER_H


#include "sriCommDefine.h"
#include "sriCommTCPClient.h"
#include "sriCommATParser.h"
#include "sriCommM8218Parser.h"

class CSRICommManager
{
public:
	CSRICommManager();
	~CSRICommManager();

	bool Init(const std::string &ipRemote, int portRemote, SRICommM8218CallbackFunction callback);
	bool Run();
	bool Stop();

	bool SendCommand(std::string command, std::string parames);
	
	bool OnNetworkFailure(std::string infor);
	bool OnCommACK(std::string command);
	bool OnCommM8218(float fx, float fy, float fz, float mx, float my, float mz);
private:
	CSRICommTCPClient mTCPClient;
	CSRICommATParser mATParser;
	CSRICommM8218Parser mM8218Parser;
	bool mIsGetACK;
	std::string mCommandACK;
	std::string mParamesACK;

};

#endif

