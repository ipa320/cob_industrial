#pragma once
#include "Poco/Net/SocketAddress.h"
#include "StringEthernetClient.h"
#include "XmlStringHandler.h"

#include "Poco/Mutex.h"
#include "Poco/Condition.h"
#include <boost/shared_ptr.hpp>

#include "KukaAxis.h"

namespace confirmationTag
{
	enum  {WAITING = 10, FINISHED = 20, REJECTED = 30, KRL_ERROR = 40}; 
}

namespace msgID
{
	enum { EMPTY_SLOT = -1 };
}


class SubID
{
public:
	enum subIDs	{	TestCell_A = 10, StackCell_A = 11, PlacePackage_A = 12, DelieverBattery_A = 13, Pose_A = 14, TestAndStack_A = 15,
		TestCell_B = 20, StackCell_B = 21, PlacePackage_B = 22, DelieverBattery_B = 23, Pose_B = 24, TestAndStack_B = 25,
		SwitchFromAtoB = 30, SwitchFromBtoA = 40, FlushBuffer = 50, EchoTest = 66, 
		MovePTP_AXIS = 100, 
		MoveLIN_FRAME = 201,
		test1 = 111, test2 = 112, test3= 113,
		EmptySlot = -1,
		Pause = 0,
		Resume = 1
	};
};

typedef boost::shared_ptr<StringEthernetClient> StringEthernetClientPtr;
typedef boost::shared_ptr<XmlStringHandler> XmlStringHandlerPtr;	

class KukaEthernetClient
{
	//typedef boost::shared_ptr<KukaEthernetClient> KukaEthernetClientPtr;

	class SenderRunnable: public Poco::Runnable
	{
	private:
		KukaEthernetClient* _parent;
		bool	_run;
		bool	_pause;

	public:
		SenderRunnable(KukaEthernetClient* parent);
		~SenderRunnable();
		void start();
		void stop();
		bool isRunning();

		virtual void run();
	};

	typedef boost::shared_ptr<SenderRunnable> SenderRunnablePtr;
	typedef boost::shared_ptr<Thread> ThreadPtr;

	typedef boost::function<void (int)> IntermediateCallbackFunctionPtr;

public:
	enum
	{
		MAX_ACTIVE_MESSAGES = 3
	};


	struct commandMessage
	{
		int messageID;
		std::string message;
	};


public:
	KukaEthernetClient();
	~KukaEthernetClient();

	bool Initialize(std::string pathKrlEthernet, const SocketAddress& sa);
	void setCallbackFcn(IntermediateCallbackFunctionPtr callback);

	StringEthernetClientPtr getEthernetClient();
	XmlStringHandlerPtr getXmlhandler();

	KukaFrame getCurrentFrame();
	void setCurrentFrame(KukaFrame frame);

	void pause();
	void start();

	int getMessageID();
	string int2String(int value);
	string float2String(float value);

	void addMessage(const commandMessage& message, bool flush = false);
	void addMessage(int msgID, SubID::subIDs subID, KukaAxis axis, float vel, bool flush = false);
	void addMessage(int msgID, SubID::subIDs subID, KukaFrame frame, float vel, bool flush = false);
	void addMessage(int msgID, SubID::subIDs subID, bool flush = false);
	void addMessage(int msgID, SubID::subIDs subID, int param1, bool flush = false);
	void addMessage(int msgID, SubID::subIDs subID, int param1, int param2, bool flush = false);
	void addMessage(int msgID, SubID::subIDs subID, int param1, int param2, KukaAxis axis, KukaFrame frame, float linVel, float axVel, bool flush = false);


	void flushBuffer(int msgID);
	void pauseMsg(int msgID);
	void resumeMsg(int msgID);

	void movePTP(int msgID, KukaAxis axis, float vel);
	void movePTP(int msgID, float a1, float a2, float a3, float a4, float a5, float a6, float vel);
	void moveLIN(int msgID, KukaFrame frame, float vel);

private:

	KukaFrame currentFrame;
	int messageCallBack(std::string data);
	void sendNextMessage();
	bool checkRepliedMsgID(int msgID, list<commandMessage>& list);

	bool checkSync(int* slot, int count, int recvPointer, int runPointer);
	commandMessage createNewCommandMessage(int msgID, SubID::subIDs subID, int param1, int param2, KukaAxis axis, KukaFrame frame, float linVel, float axVel);

private:
	IntermediateCallbackFunctionPtr _intermediateCallbackFunction;

	int _messageCount;
	bool _isConnected;
	bool _isXMLInit;
	StringEthernetClientPtr _ethernetClient;
	XmlStringHandlerPtr _xmlHandler;

	SenderRunnablePtr	_senderWorker;
	ThreadPtr			_workerThread;

	Poco::Condition _listPendingNotEmptyCondition;
	Poco::Mutex				_messageMutex;
	Poco::Mutex	_frameMutex;

	int						_activeMessages;
	list<commandMessage>	_messageActiveList;
	list<commandMessage>	_messageDoneList;
	list<commandMessage>	_messagePendingList;

};
