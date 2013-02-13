#pragma once
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <sstream>
#include <boost/thread.hpp>

using boost::asio::ip::udp;

/* Definitions & Einstellungen */

// If defined, an information on every parsed message with IPOC count and axis position is
// shown on the console
//#define PARSE_INFORMATION 

// This parameter specifies that every x-th message is parsed and written into the variable
// RobPosG
#define PARSE_FREQUENCY 1
#define IPOC_HZ 250

/* Definitions & Einstellungen ENDE */
#ifdef TIMING
#include <ctime>
#endif

class RSIConnector {
public:
	/* Data formats */
	struct RobotPosition {
			RobotPosition() :
			A1(0.0f), A2(0.0f), A3(0.0f), A4(0.0f), A5(0.0f), A6(0.0f) {}
		float A1;
		float A2;
		float A3;
		float A4;
		float A5;
		float A6;
		int timestamp;
	};
	
	struct RobotCorrection {
		RobotCorrection() :
	dX(0.0f), dY(0.0f), dZ(0.0f), dA(0.0f), dB(0.0f), dC(0.0f) {}
		float dX;
		float dY;
		float dZ;
		float dA;
		float dB;
		float dC;
	};

	struct AxisCorrection {
		AxisCorrection() :
			dA1(0.0f), dA2(0.0f), dA3(0.0f), dA4(0.0f), dA5(0.0f), dA6(0.0f) {}
		float dA1;
		float dA2;
		float dA3;
		float dA4;
		float dA5;
		float dA6;
	};

public:
	RSIConnector(bool correctAxis = true);
	~RSIConnector(void);
	int connect(std::string own_ip, int udp_port);
	void start();
	void stop();

	RobotPosition GetRobPos();
	void SetRobCorrection(const RobotCorrection&);
	void SetAxisCorrection(const AxisCorrection&);
	int GetIpoCount();

private:
	void SetRobPos(const RobotPosition&);
	RobotCorrection GetRobCorrection();
	AxisCorrection GetAxisCorrection();

private:
	boost::asio::io_service _io_service;
	udp::socket* _socket;
	bool _isConnected;

	void work();
	void parseXML();
	std::string extractIPOC(const std::string& receive);

	bool _run;
	int _ipoCount;
	boost::thread* _worker;
	boost::mutex _mutexStringParser;
	boost::mutex _mutexRobCor;
	boost::mutex _mutexRobPos;
	bool _correctAxis;
	RobotCorrection _RKorr;
	AxisCorrection _AKorr;
	RobotPosition _robPosG;

	std::string _dataFromRobot;

};

