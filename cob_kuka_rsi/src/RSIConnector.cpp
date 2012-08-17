#include "cob_kuka_rsi/RSIConnector.h"

RSIConnector::RSIConnector(std::string own_ip, int udp_port,
		bool isCorrectingAxes) {
	memset(&_robPosG, 0, sizeof(_robPosG));
	memset(&_RKorr, 0, sizeof(_RKorr));
	memset(&_AKorr, 0, sizeof(_AKorr));
	_correctAxis = isCorrectingAxes;

	boost::system::error_code error;
	_socket = new udp::socket(_io_service);
	_socket->open(udp::v4());
	_socket->bind(udp::endpoint(boost::asio::ip::address::from_string(own_ip),
			udp_port), error);
	if (error) {
		std::cerr << "Error while binding socket: " << error.message()
				<< std::endl;
	}
	_run = false;
}

RSIConnector::~RSIConnector(void) {
}

/* Data Parser */
void RSIConnector::parseXML() {

#ifdef TIMING
	clock_t beginn = getMilliSecs();
#endif

	try {
		_mutexStringParser.lock();
		RobotPosition robPos;
		sscanf(
				_dataFromRobot.c_str(),
				"<Rob Type=\"KUKA\"><AIPos A1=\"%f\" A2=\"%f\" A3=\"%f\" A4=\"%f\" A5=\"%f\" A6=\"%f\"/><IPOC>%d</IPOC></Rob>",
				&robPos.A1, &robPos.A2, &robPos.A3, &robPos.A4, &robPos.A5,
				&robPos.A6, &robPos.timestamp);
		_mutexStringParser.unlock();

		SetRobPos(robPos);

#ifdef PARSE_INFORMATION
		std::cout << "IPOC: " << robPos.timestamp << "\tA1: " << robPos.A1 << " A2: " << robPos.A2 << " A3: " << robPos.A3 << " A4: " << robPos.A4 << " A5: " << robPos.A5 << " A6: " << robPos.A6 << std::endl;
#endif
	} catch (std::exception e) {
		std::cerr << "Error while reading xml: " << e.what() << std::endl;
		getchar();
	}

#ifdef TIMING
	std::cout << "It took " << (getMilliSecs()-beginn) << " ms to parse" << std::endl << std::endl;
#endif
}

/* Standard code */
std::string RSIConnector::extractIPOC(const std::string& receive) {
	int startdummy = receive.rfind("<IPOC>") + 6;
	int stopdummy = receive.rfind("</IPOC>");
	std::string Ipocount = receive.substr(startdummy, stopdummy - startdummy);
	return Ipocount;
}

void RSIConnector::work() {
	RobotPosition robPos = { 0 };
	_ipoCount = 0;

	while (_run) {
		try {
			boost::array<char, 1024> recv_buf;
			udp::endpoint remote_endpoint;
			boost::system::error_code error;

			// Get Correction before time critical receiption of message by KUKA
			AxisCorrection axCor;
			RobotCorrection robCor = { 0 };
			if (_correctAxis) {
				axCor = GetAxisCorrection();
			} else {
				robCor = GetRobCorrection();
			}

			//Receive message by KUKA RSI
			std::ostringstream conv;
			_socket->receive_from(boost::asio::buffer(recv_buf),
					remote_endpoint, 0, error);

			// Convert to member string stream
			conv << recv_buf.data();
			// Lock conversion because parser Thread (from last cycle) might work with it
			_mutexStringParser.lock();
			_dataFromRobot = conv.str();
			_mutexStringParser.unlock();

			// Extract IPO counter for reply
			std::string timestamp = extractIPOC(conv.str());

			std::stringstream messageStream;
			if (_correctAxis) {
				messageStream << "<Sen Type=\"ImFree\">\n<AK A1=\""
						<< axCor.dA1 << "\" A2=\"" << axCor.dA2 << "\" A3=\""
						<< axCor.dA3 << "\" A4=\"" << axCor.dA4 << "\" A5=\""
						<< axCor.dA5 << "\" A6=\"" << axCor.dA6
						<< "\" />\n<IPOC>" + timestamp + "</IPOC>\n</Sen>\n";
			} else {
				messageStream << "<Sen Type=\"ImFree\">\n<RKorr X=\""
						<< robCor.dX << "\" Y=\"" << robCor.dY << "\" Z=\""
						<< robCor.dZ << "\" A=\"" << robCor.dA << "\" B=\""
						<< robCor.dB << "\" C=\"" << robCor.dC
						<< "\" />\n<IPOC>" + timestamp + "</IPOC>\n</Sen>\n";
			}
			std::string message = messageStream.str();

			boost::system::error_code ignored_error;
			// Send reply
			_socket->send_to(boost::asio::buffer(message), remote_endpoint, 0,
					ignored_error);

			// Parse message to extract current position
			// Only parse every x-th message ... 
			if ((_ipoCount % PARSE_FREQUENCY) == 0) {
				boost::thread myParser(boost::bind(&RSIConnector::parseXML,
						this));
			}
			_ipoCount++;

			//std::cout << "Sent Message: " << message << std::endl;
			conv.clear();
			messageStream.clear();
		} catch (std::exception ex) {
			std::cerr << "Error during Socket Communication: " << ex.what()
					<< std::endl;
		}
	}
}

void RSIConnector::start() {
	_run = true;
	_worker = new boost::thread(boost::bind(&RSIConnector::work, this));
}

void RSIConnector::stop() {
	_run = false;
	_socket->close();
	_worker->join();
}

RSIConnector::RobotPosition RSIConnector::GetRobPos() {
	RobotPosition robPos;
	_mutexRobPos.lock();
	robPos = _robPosG;
	_mutexRobPos.unlock();
	return robPos;
}

void RSIConnector::SetRobCorrection(const RobotCorrection& robCor) {
	_mutexRobCor.lock();
	_RKorr = robCor;
	_mutexRobCor.unlock();
}

void RSIConnector::SetRobPos(const RobotPosition& robPos) {
	_mutexRobPos.lock();
	_robPosG = robPos;
	_mutexRobPos.unlock();
}

int RSIConnector::GetIpoCount() {
	return _ipoCount;
}

RSIConnector::RobotCorrection RSIConnector::GetRobCorrection() {
	RobotCorrection robCor = { 0 };
	_mutexRobCor.lock();
	robCor = _RKorr;
	_mutexRobCor.unlock();
	return robCor;
}

RSIConnector::AxisCorrection RSIConnector::GetAxisCorrection() {
	AxisCorrection axCor;
	_mutexRobCor.lock();
	axCor = _AKorr;
	_mutexRobCor.unlock();
	return axCor;
}

void RSIConnector::SetAxisCorrection(const AxisCorrection& axCor) {
	_mutexRobCor.lock();
	_AKorr = axCor;
	_mutexRobCor.unlock();
}

