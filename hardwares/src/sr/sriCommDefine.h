#ifndef SRI_COMM_DEFINE_H
#define SRI_COMM_DEFINE_H

//#define IS_WINDOWS_OS  0


#ifdef  IS_WINDOWS_OS
	#include <stdio.h>
	#include <tchar.h>
	#include <string.h>
	#include <iostream>

	#include <thread> 
	#include <mutex>

	#include <ctime>
	
	#include <vector>

	#include <winsock2.h>
	#include <WS2tcpip.h> 

	#include <SDKDDKVer.h>

	typedef std::function<bool(std::string)> SRICommNetworkFailureCallbackFunction;
	typedef std::function<bool(std::string)> SRICommATCallbackFunction;
	typedef std::function<bool(float fx, float fy, float fz, float mx, float my, float mz)> SRICommM8218CallbackFunction;

#else
	#include<stdio.h>
	#include<stdlib.h>
	#include<string.h>
	#include <iostream>

	#include<errno.h>
	#include<sys/types.h>
	#include<sys/socket.h>
	#include <sys/ioctl.h>
	#include<netinet/in.h>
	#include<arpa/inet.h>
	#include<netinet/tcp.h>
	#include<unistd.h>
	#include <functional>
	#include <thread>
	#include <mutex>

	#include <ctime>
	#include <sys/time.h>        //gettimeofday()

	#include <vector>
	#include <algorithm>


	typedef std::function<bool(std::string)> SRICommNetworkFailureCallbackFunction;
	typedef std::function<bool(std::string)> SRICommATCallbackFunction;
	typedef std::function<bool(float fx, float fy, float fz, float mx, float my, float mz)> SRICommM8218CallbackFunction;

	typedef  unsigned char BYTE;

	#ifndef max
	#define max(a,b)            (((a) > (b)) ? (a) : (b))
	#endif

	#ifndef min
	#define min(a,b)            (((a) < (b)) ? (a) : (b))
	#endif

#endif


#endif
