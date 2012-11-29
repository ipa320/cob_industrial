#include "StringEthernetClient.h"

//********** Worker *************//
StringEthernetClient::WorkerRunnable::WorkerRunnable(const SocketAddress& sa):
	_pBuffer(new char[BUFFER_SIZE])
{
	_sa = sa;
	_run = true;
	_sendPending = false;
	_isConnected = false;
	
	try
	{
		_socket.reset(new StreamSocket(_sa));
		_isConnected = true;
	}
	catch (...)
	{
		logs.error() << "Cannot connect to TCP server" << endl;
	}

}

StringEthernetClient::WorkerRunnable::~WorkerRunnable()
{
	_mutex.unlock();
}

void StringEthernetClient::WorkerRunnable::start()
{
	_run = true;
	_sendPending = false;
	_mutex.unlock();
}

bool StringEthernetClient::WorkerRunnable::isRunning()
{
	return _run;
}

bool StringEthernetClient::WorkerRunnable::isConnected()
{
	return _isConnected;
}
void StringEthernetClient::WorkerRunnable::stop()
{
	_run = false;
	_socket->close();
	_isConnected = false;
}

void StringEthernetClient::WorkerRunnable::reconnect()
{
	logs.warning() << "Trying to reconnect..." << endl;
	_socket->close();
	_isConnected = false;
	try
	{
		_socket.reset(new StreamSocket(_sa));
		_isConnected = true;
	}
	catch (...)
	{
		logs.error() << "Cannot connect to TCP server" << endl;
		Poco::Thread::sleep(500);
	}
}	

void StringEthernetClient::WorkerRunnable::setCallbackFcn(CallbackFunctionPtr callback)
{
	_callback = callback;
}

int StringEthernetClient::WorkerRunnable::send(string message)
{
	_mutex.lock();
	int queue = _sendList.size();
	_sendList.push_back(message);
	_mutex.unlock();
	
	_sendPending = true;
	return queue;
}

void StringEthernetClient::WorkerRunnable::run()
{
	while (_run)
	{
		if (!_isConnected)
		{
			logs.warning() << "Communication error" << endl;
			Poco::Thread::sleep(500);
			reconnect();
		}
		
		if (_isConnected)
		{
			try
			{
				if (_sendPending)
				{
					if (_socket->poll(100, Socket::SELECT_WRITE))
					{
						_mutex.lock();
						_socket->sendBytes(_sendList.begin()->data(), _sendList.begin()->length());
						_sendList.pop_front();
						if (_sendList.size() == 0)
						{
							_sendPending = false;
						}
						_mutex.unlock();
					}
				}
				else
				{
					if (_socket->poll(100, Socket::SELECT_READ))
					{
						int n = _socket->receiveBytes(_pBuffer, BUFFER_SIZE);
						if (n > 0)
						{
							int result = _callback(string(_pBuffer, n));
						}
					}
				}
			}
			catch (...)
			{
				logs.warning() << "Communication error" << endl;
				//reconnect();
			}
		} 
		else
		{
			logs.warning() << "Communication error" << endl;
			//reconnect();
		}
	}
}

//*************** Ethernet Client ***************//
StringEthernetClient::StringEthernetClient():
	_callback(NULL)
{
	
}
	
StringEthernetClient::~StringEthernetClient()
{
	disconnect();
}

void StringEthernetClient::setCallbackFcn(boost::function<int (std::string)> callback)
{
	_callback = callback;
	if (!!_workerRunnable)
	{
		_workerRunnable->setCallbackFcn(callback);
	}
}

int StringEthernetClient::connect(const SocketAddress& sa)
{
	_workerThread.reset(new Thread);
	_workerRunnable.reset(new WorkerRunnable(sa));
	if (_callback != NULL)
	{
		_workerRunnable->setCallbackFcn(_callback);
	}
	
	_workerThread->start(*_workerRunnable);
	return _workerRunnable->isConnected();
}

int StringEthernetClient::disconnect()
{
	_workerRunnable->stop();
	_workerThread->join();
	return 0;
}

int StringEthernetClient::send(std::string data)
{
	return _workerRunnable->send(data);
}

