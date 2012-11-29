#define SENDING_SLEEPING_TIME_MS 1000

#pragma once
#include "KukaEthernetClient.h"
#include "XmlDefines.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <functional>
#include "boost/lexical_cast.hpp"


KukaEthernetClient::SenderRunnable::SenderRunnable(KukaEthernetClient* parent)
{
	_parent = parent;
	_run = true;
	_pause = false;
}

KukaEthernetClient::SenderRunnable::~SenderRunnable()
{

}

bool KukaEthernetClient::SenderRunnable::isRunning()
{
	return _run;
}

void KukaEthernetClient::SenderRunnable::stop()
{
	_pause = true;
}

void KukaEthernetClient::SenderRunnable::start()
{
	_pause = false;
}

void KukaEthernetClient::SenderRunnable::run()
{
	Poco::Mutex mutex;
	mutex.lock();
	while (_run)
	{
		if (!_pause)
		{
			_parent->_messageMutex.lock();
			if (_parent->_messagePendingList.size() == 0)
			{
				_parent->_messageMutex.unlock();
				mutex.lock();
				_parent->_listPendingNotEmptyCondition.wait(mutex);
			}
			else
			{
				_parent->_messageMutex.unlock();		
				_parent->sendNextMessage();
				Poco::Thread::sleep(SENDING_SLEEPING_TIME_MS);
			}
		}
		else
		{
			Poco::Thread::sleep(250);
		}
	}
}

//******************* KUKA Client *************************//

KukaEthernetClient::KukaEthernetClient()
	:	_isConnected(false),
	_isXMLInit(false),
	_activeMessages(0),
	_messageCount(0),
	_intermediateCallbackFunction(NULL)
{
	_ethernetClient.reset(new StringEthernetClient());
	_xmlHandler.reset(new XmlStringHandler());
	_workerThread.reset(new Thread);
	_senderWorker.reset(new SenderRunnable(this));
	_workerThread->start(*_senderWorker);

	_messageMutex.unlock();
}

KukaEthernetClient::~KukaEthernetClient()
{
	_messageMutex.unlock();
}


void KukaEthernetClient::setCallbackFcn(IntermediateCallbackFunctionPtr callback)
{
	_intermediateCallbackFunction = callback;
}

void KukaEthernetClient::pause()
{
	_senderWorker->stop();
}

void KukaEthernetClient::start()
{
	_senderWorker->start();
}


bool KukaEthernetClient::checkRepliedMsgID(int messageID, list<commandMessage>& list)
{
	bool result = false;
	if (list.size() > 0)
	{
		result = (messageID == list.begin()->messageID);
		if (!result)
		{
#ifdef NO_LOG_AVAILABLE
			std::cout << "Message ID compare failed (Exp: " << list.begin()->messageID << ", Recv: " << messageID << std::endl;
#else
			logs.warning() << "Message ID compare failed (Exp: " << list.begin()->messageID << ", Recv: " << messageID << std::endl;
#endif
		}
	}
	else
	{
#ifdef NO_LOG_AVAILABLE
		std::cout << "Message ID compare failed (List was empty)" << std::endl;
#else
		logs.warning() << "Message ID compare failed (List was empty)" << std::endl;
#endif
	}
	return result;
}

int KukaEthernetClient::messageCallBack(std::string data)
{
	_xmlHandler->parseDataFromRobot(data);

#ifdef NO_LOG_AVAILABLE
	std::cout << "Received Message:" << endl << data << endl;
#else
	logs.warning() << "Received Message:" << endl << data << endl;
#endif

	int parsingError = -1;
	int resultError = 0;

	int messageID = _xmlHandler->getIntFromRobot(TAG_RECV_MSG_ID, parsingError);
	int confirmation = _xmlHandler->getIntFromRobot(TAG_RECV_CONFIRM, parsingError);
	int subID = _xmlHandler->getIntFromRobot(TAG_RECV_SUB_ID, parsingError);


	// for debugging purposes (Check Sync between KRC and PC)
	int runPointer = _xmlHandler->getIntFromRobot(TAG_RECV_RUN_POINTER, parsingError);
	int recvPointer = _xmlHandler->getIntFromRobot(TAG_RECV_RECV_POINTER, parsingError);
	int slot[3] = {-2};
	slot[0] = _xmlHandler->getIntFromRobot(TAG_RECV_SLOT1, parsingError);
	slot[1] = _xmlHandler->getIntFromRobot(TAG_RECV_SLOT2, parsingError);
	slot[2] = _xmlHandler->getIntFromRobot(TAG_RECV_SLOT3, parsingError);

	if (!checkSync(&slot[0], 3, recvPointer, runPointer))
	{
#ifdef NO_LOG_AVAILABLE
		std::cout <<  "KukaEthernetClient and KRL seem out of Sync!" << endl;
#else
		logs.critical() <<  "KukaEthernetClient and KRL seem out of Sync!" << endl;
#endif
	}

	if(!parsingError)
	{
		_messageMutex.lock();
		if (checkRepliedMsgID(messageID, _messageActiveList))	// messageID wurde gefunden und vom Kuka durchgeführt
		{
			_messageDoneList.push_back(*_messageActiveList.begin());
			_messageActiveList.pop_front();
			_activeMessages--;

			if (_intermediateCallbackFunction != NULL)
			{
				_intermediateCallbackFunction(messageID);
			}
		}
		else
		{
#ifdef NO_LOG_AVAILABLE
			std::cout << "Message " << messageID << ", SubID: " << subID << " was performed BUT cannot be found to be sent before! (activeList)" << endl;
#else
			logs.warning() <<  "Message " << messageID << ", SubID: " << subID << " was performed BUT cannot be found to be sent before! (activeList)" << endl;
#endif
			resultError = 3;
		}
		_messageMutex.unlock();
	}
	else
	{
#ifdef NO_LOG_AVAILABLE
		std::cout << "Parsing Error. Message: " << messageID << "ignored." << endl;
#else
		logs.warning() << "Parsing Error. Message: " << messageID << "ignored." << endl;
#endif
		resultError = -1;
	}


	return resultError;

	//HALT
	//
	//
	//	if (!error1 && !error2)
	//	{
	//		if (subID != SubID::EmptySlot)
	//		{
	//			switch (confirmation)
	//			{
	//			case confirmationTag::WAITING:
	//				_messageMutex.lock();
	//				if (checkRepliedMsgID(messageID, _messageUnconfirmedList))
	//				{
	//					_messageActiveList.push_back(*_messageUnconfirmedList.begin());
	//					_messageUnconfirmedList.pop_front();
	//	#ifdef NO_LOG_AVAILABLE
	//					std::cout << "Message " << messageID << " was accepted by KUKA and is waiting to be performed" << endl;
	//	#else
	//					logs.debug() << "Message " << messageID << " was accepted by KUKA and is waiting to be performed" << endl;
	//	#endif
	//				}
	//				else
	//				{
	//	#ifdef NO_LOG_AVAILABLE
	//					std::cout << "Message " << messageID << ", SubID: " << subID << " was accepted by KUKA BUT cannot be found to be sent before! (unconfirmedList)" << endl;
	//	#else
	//					logs.warning() << "Message " << messageID << ", SubID: " << subID << " was accepted by KUKA BUT cannot be found to be sent before! (unconfirmedList)" << endl;
	//	#endif
	//					resError = 3;
	//				}
	//				_messageMutex.unlock();
	//				break;
	//			case confirmationTag::FINISHED:
	//				_messageMutex.lock();
	//				if (checkRepliedMsgID(messageID, _messageActiveList))
	//				{
	//						_messageActiveList.pop_front();
	//						_activeMessages--;
	//	#ifdef NO_LOG_AVAILABLE
	//					cout << "Message " << messageID << " was performed and is deleted" << endl;
	//	#else
	//					logs.debug() << "Message " << messageID << " was performed and is deleted" << endl;
	//	#endif
	//				}
	//				else
	//				{
	//	#ifdef NO_LOG_AVAILABLE
	//					std::cout << "Message " << messageID << ", SubID: " << subID << " was performed BUT cannot be found to be sent before! (activeList)" << endl;
	//	#else
	//					logs.warning() <<  "Message " << messageID << ", SubID: " << subID << " was performed BUT cannot be found to be sent before! (activeList)" << endl;
	//	#endif
	//					resError = 3;
	//				}
	//
	//				if (_intermediateCallbackFunction != NULL)
	//				{
	//					if (messageID != msgID::EMPTY_SLOT)
	//					{
	//						_intermediateCallbackFunction(messageID);
	//					}
	//				}
	//				_messageMutex.unlock();
	//
	//				break;
	//	//		case confirmationTag::REJECTED:
	//	//#ifdef NO_LOG_AVAILABLE
	//	//			std::cout << "Message " << messageID  << ", SubID: " << subID << " has been rejected by KUKA" << endl;
	//	//#else
	//	//			logs.warning() << "Message " << messageID  << ", SubID: " << subID << " has been rejected by KUKA" << endl;
	//	//#endif
	//	//		
	//	//			_messagePendingList.push_front(*_messageUnconfirmedList.begin());
	//	//			_messageUnconfirmedList.pop_front();
	//
	//	//			break;
	//			case confirmationTag::KRL_ERROR:
	//	#ifdef NO_LOG_AVAILABLE
	//				std::cout << "Message " << messageID  << ", SubID: " << subID << " ConfirmationTag is KRL_ERROR. Some KRL Iteration Error occured! Message ignored!" << endl;
	//	#else
	//				logs.warning() << "Message " << messageID  << ", SubID: " << subID << " ConfirmationTag is KRL_ERROR. Some KRL Iteration Error occured! Message ignored!" << endl;
	//	#endif
	//				resError = 3;
	//				break;
	//			}
	//		}
	//		else
	//		{
	//			// SubId::EmptySlot occured! Ignore message
	//	#ifdef NO_LOG_AVAILABLE
	//				std::cout << "Message " << messageID  << ", SubID: " << subID << ". Some KRL Iteration Error occured! Message ignored!" << endl;
	//	#else
	//				logs.warning() << "Message " << messageID  << ", SubID: " << subID << ". Some KRL Iteration Error occured! Message ignored!" << endl;
	//	#endif
	//			resError = 3;
	//		}
	//	}
	//	else
	//	{
	//#ifdef NO_LOG_AVAILABLE
	//		std::cout << "Parsing Error. Message: " << messageID << "ignored." << endl;
	//#else
	//		logs.warning() << "Parsing Error. Message: " << messageID << "ignored." << endl;
	//#endif
	//		resError = -1;
	//	}

	return 0;
}

bool KukaEthernetClient::Initialize(std::string pathKrlEthernet, const SocketAddress& sa)
{

	boost::function<int (std::string)> f3( boost::bind( &KukaEthernetClient::messageCallBack, this, _1 ) );
	_ethernetClient->setCallbackFcn(f3);

	logs.debug() << "Connecting KukaEthernetClient" << endl;
	_isConnected = _ethernetClient->connect(sa);

	logs.debug() << "Initializing XmlStringHandler" << endl;
	_isXMLInit = !_xmlHandler->initAll(pathKrlEthernet);
	return (_isConnected && _isXMLInit);
}

StringEthernetClientPtr KukaEthernetClient::getEthernetClient()
{
	return _ethernetClient;
}

XmlStringHandlerPtr KukaEthernetClient::getXmlhandler()
{
	return _xmlHandler;
}

void KukaEthernetClient::addMessage(const commandMessage& i_message, bool flush)
{
	_messageMutex.lock();
	if (!flush)
	{
		_messagePendingList.push_back(i_message);
	}
	else
	{
		_messagePendingList.clear();
		_messagePendingList.push_back(i_message);
		_messageActiveList.resize(1);
		_activeMessages = _messageActiveList.size();
	}
	_messageMutex.unlock();

	_listPendingNotEmptyCondition.broadcast();
}

bool KukaEthernetClient::checkSync(int* slot, int count, int recvPointer, int runPointer)
{
	if (_messageActiveList.size() > count)
	{
#ifdef NO_LOG_AVAILABLE
		std::cout <<  "More messages in active list than defined!" << endl;
#else
		logs.critical() <<  "More messages in active list than defined!" << endl;
#endif  
		return false;
	}

	string info = "";

	bool result = true;
	list<commandMessage>::iterator iterActivList = _messageActiveList.begin();
	while (iterActivList != _messageActiveList.end())
	{
		result &= (iterActivList->messageID == slot[runPointer-1]);
		info.append(boost::lexical_cast<string>(iterActivList->messageID) + "<c-r>" + boost::lexical_cast<string>(slot[runPointer-1]) + "; ");
		iterActivList++;
		//wrap around
		if(runPointer < count)
		{
			runPointer++;
		}
		else
		{
			runPointer = 1;
		}
	}

	if (!result)
	{
#ifdef NO_LOG_AVAILABLE
		std::cout <<  "Sync Problem! ["<< info << "]" << endl;
#else
		logs.critical() <<  "Sync Problem! ["<< info << "]" << endl;
#endif
	}
	else
	{
#ifdef NO_LOG_AVAILABLE
		std::cout <<  "Sync OK! ["<< info << "]" << endl;
#else
		logs.debug() <<  "Sync OK! ["<< info << "]" << endl;
#endif
	}
	return result;
}

int KukaEthernetClient::getMessageID()
{
	int msgID = _messageCount;

	if (_messageCount < INT_MAX)
	{
		_messageCount++;
	}
	else
	{
		_messageCount = 0;
	}

	return msgID;
}

string KukaEthernetClient::int2String(int value)
{
	char buffer[50];
	int n;
	n = sprintf(buffer, "%d", value);
	return string(buffer, n);
}

string KukaEthernetClient::float2String(float value)
{
	char buffer[50];
	int n;
	n = sprintf(buffer, "%f", value);
	return string(buffer, n);
}

void KukaEthernetClient::sendNextMessage()
{
	_messageMutex.lock();
	if (_messagePendingList.size() > 0
		&& _activeMessages < MAX_ACTIVE_MESSAGES)
	{	
#ifdef NO_LOG_AVAILABLE
		std::cout << "Try to send messageID " << _messagePendingList.begin()->messageID << endl;
#else
		logs.warning() << "Try to send messageID " << _messagePendingList.begin()->messageID << endl;
#endif
		_activeMessages++;

		_ethernetClient->send(_messagePendingList.begin()->message);

		_messageActiveList.push_back(*_messagePendingList.begin());
		_messagePendingList.pop_front();
	}
	_messageMutex.unlock();
}

//************************** Main Add Message Function *************************//
void KukaEthernetClient::addMessage(int msgID, SubID::subIDs subID, int param1, int param2, KukaAxis axis, KukaFrame frame, float linVel, float axVel, bool flush)
{
	commandMessage commandMsg;
	commandMsg = createNewCommandMessage(msgID, subID, param1, param2, axis, frame, linVel, axVel);

#ifdef NO_LOG_AVAILABLE
	std::cout << "Adding messageID " << commandMsg.messageID << ",  string = " << endl << commandMsg.message << endl;
#else
	logs.debug() << "Adding messageID " << commandMsg.messageID << ",  string = " << endl << commandMsg.message << endl;
#endif
	addMessage(commandMsg, flush);
}
//******************************************************************************//

void KukaEthernetClient::addMessage(int msgID, SubID::subIDs subID, KukaFrame frame, float linVel, bool flush)
{
	KukaAxis axis;
	addMessage(msgID, subID, 0, 0, axis, frame, linVel, 0, flush);
}

void KukaEthernetClient::addMessage(int msgID, SubID::subIDs subID, KukaAxis axis, float axVel, bool flush)
{
	KukaFrame frame;
	addMessage(msgID, subID, 0, 0, axis, frame, 0, axVel, flush);
}

void KukaEthernetClient::addMessage(int msgID, SubID::subIDs subID, int param1, bool flush)
{
	KukaAxis axis;
	KukaFrame frame;
	addMessage(msgID, subID, param1, 0, axis, frame, 0, 0, flush);
}

void KukaEthernetClient::addMessage(int msgID, SubID::subIDs subID, int param1, int param2, bool flush)
{
	KukaAxis axis;
	KukaFrame frame;
	addMessage(msgID, subID, param1, param2, axis, frame, 0, 0, flush);
}

void KukaEthernetClient::addMessage(int msgID, SubID::subIDs subID, bool flush)
{
	KukaAxis axis;
	KukaFrame frame;
	addMessage(msgID, subID, 0, 0, axis, frame, 0, 0, flush);
}


//********************** specialized messages SILIA ****************************//
void KukaEthernetClient::testCell(int msgID, int posU, int posG, MRKZelle::Area coopArea)
{
	if (coopArea == MRKZelle::Area_A)
	{
		addMessage(msgID, SubID::subIDs::TestCell_A, posU, posG);
	}
	else if (coopArea == MRKZelle::Area_B) 
	{
		addMessage(msgID, SubID::subIDs::TestCell_B, posU, posG);// (msgID, 20, posU, posG);
	}
	else
	{
#ifdef NO_LOG_AVAILABLE
		std::cout << "Unknown CoopArea ID: " << coopArea;
#else
		logs.error() << "Unknown CoopArea ID: " << coopArea;
#endif
		return;
	}
}

void KukaEthernetClient::stackCell(int msgID, int posG, int posZ, MRKZelle::Area coopArea)
{	
	if (coopArea == MRKZelle::Area_A)
	{
		addMessage(msgID, SubID::subIDs::StackCell_A, posG, posZ);
	}
	else if (coopArea == MRKZelle::Area_B) 
	{
		addMessage(msgID,  SubID::subIDs::StackCell_B, posG, posZ);
	}
	else
	{
#ifdef NO_LOG_AVAILABLE
		std::cout << "Unknown CoopArea ID: " << coopArea;
#else
		logs.error() << "Unknown CoopArea ID: " << coopArea;
#endif
		return;
	}
}

void KukaEthernetClient::testAndStackCell(int msgID, int posU, int posZ, MRKZelle::Area coopArea)
{	
	if (coopArea == MRKZelle::Area_A)
	{
		addMessage(msgID, SubID::subIDs::TestAndStack_A, posU, posZ);
	}
	else if (coopArea == MRKZelle::Area_B) 
	{
		addMessage(msgID,  SubID::subIDs::TestAndStack_B, posU, posZ);
	}
	else
	{
#ifdef NO_LOG_AVAILABLE
		std::cout << "Unknown CoopArea ID: " << coopArea;
#else
		logs.error() << "Unknown CoopArea ID: " << coopArea;
#endif
		return;
	}
}

void KukaEthernetClient::placePackage(int msgID, MRKZelle::Area coopArea)
{
	if (coopArea == MRKZelle::Area_A)
	{
		addMessage(msgID,  SubID::subIDs::PlacePackage_A);
	}
	else if (coopArea == MRKZelle::Area_B) 
	{
		addMessage(msgID,  SubID::subIDs::PlacePackage_B);
	}
	else
	{
#ifdef NO_LOG_AVAILABLE
		std::cout << "Unknown CoopArea ID: " << coopArea;
#else
		logs.error() << "Unknown CoopArea ID: " << coopArea;
#endif
		return;
	}
}

void KukaEthernetClient::delieverBattery(int msgID, int posP, MRKZelle::Area coopArea)
{
	if (coopArea == MRKZelle::Area_A)
	{
		addMessage(msgID,  SubID::subIDs::DelieverBattery_A, posP);
	}
	else if (coopArea == MRKZelle::Area_B) 
	{
		addMessage(msgID, SubID::subIDs::DelieverBattery_B, posP);
	}
	else
	{
#ifdef NO_LOG_AVAILABLE
		std::cout << "Unknown CoopArea ID: " << coopArea;
#else
		logs.error() << "Unknown CoopArea ID: " << coopArea;
#endif
		return;
	}
}

void KukaEthernetClient::switchA_B(int msgID)
{
	addMessage(msgID, SubID::subIDs::SwitchFromAtoB); // SubID = 30
}

void KukaEthernetClient::switchB_A(int msgID)
{
	addMessage(msgID, SubID::subIDs::SwitchFromBtoA); // SubId = 40
}

void KukaEthernetClient::flushBuffer(int msgID)
{
	addMessage(msgID,  SubID::subIDs::FlushBuffer, true);
}

void KukaEthernetClient::dummyPoser(int msgID, MRKZelle::Area coopArea)
{
	if (coopArea == MRKZelle::Area_A)
	{
		addMessage(msgID,  SubID::subIDs::Pose_A);
	}
	else if (coopArea == MRKZelle::Area_B) 
	{
		addMessage(msgID, SubID::subIDs::Pose_B);
	}
	else
	{
#ifdef NO_LOG_AVAILABLE
		std::cout << "Unknown CoopArea ID: " << coopArea;
#else
		logs.error() << "Unknown CoopArea ID: " << coopArea;
#endif
		return;
	}
}

//******************************************************************************//

void KukaEthernetClient::movePTP(int msgID, float a1, float a2, float a3, float a4, float a5, float a6, float vel)
{
	KukaAxis axis(a1, a2, a3, a4, a5, a6); 
	addMessage(msgID, SubID::subIDs::MovePTP_AXIS, axis, false);
}

void KukaEthernetClient::movePTP(int msgID, KukaAxis axis, float vel)
{
	if (vel > 100.0)
	{
		logs.warning() << "Axis Movement Speed adjusted higher than 100%; Value truncated to 90%" << endl;
		vel = 90.0;
	}
	addMessage(msgID, SubID::subIDs::MovePTP_AXIS, axis, vel, false);
}

void KukaEthernetClient::moveLIN(int msgID, KukaFrame frame, float vel)
{
	if (vel > 2.0)
	{
		logs.warning() << "Linear Movement Speed adjusted higher than 2m/s; Value truncated to 1.9m/s" << endl;
		vel = 1.9;
	}
	addMessage(msgID, SubID::subIDs::MoveLIN_FRAME, frame, vel, false);
}

void KukaEthernetClient::pauseMsg( int msgID )
{
	commandMessage commandMsg;
	commandMsg = createNewCommandMessage(msgID, SubID::Pause, 0, 0, KukaAxis(), KukaFrame(), 0, 0);
	_ethernetClient->send(commandMsg.message);
}

void KukaEthernetClient::resumeMsg( int msgID )
{
	commandMessage commandMsg;
	commandMsg = createNewCommandMessage(msgID, SubID::Resume, 0, 0, KukaAxis(), KukaFrame(), 0, 0);
	_ethernetClient->send(commandMsg.message);
}

KukaEthernetClient::commandMessage KukaEthernetClient::createNewCommandMessage( int msgID, SubID::subIDs subID, int param1, int param2, KukaAxis axis, KukaFrame frame, float linVel, float axVel )
{
	_xmlHandler->setDataToRobot(TAG_SEND_SUB_ID, int2String((int)subID));
	_xmlHandler->setDataToRobot(TAG_SEND_MSG_ID, int2String(msgID));

	_xmlHandler->setDataToRobot(TAG_SEND_PARAM1, int2String(param1));
	_xmlHandler->setDataToRobot(TAG_SEND_PARAM2, int2String(param2));
	_xmlHandler->setDataToRobot(TAG_SEND_FRAME, frame);
	_xmlHandler->setDataToRobot(TAG_SEND_AXIS, axis);
	_xmlHandler->setDataToRobot(TAG_SEND_AXVEL, float2String(axVel));
	_xmlHandler->setDataToRobot(TAG_SEND_LINVEL, float2String(linVel));

	commandMessage commandMsg;
	commandMsg.messageID = msgID;
	commandMsg.message = _xmlHandler->createDataToRobotXml();

	return commandMsg;
}
