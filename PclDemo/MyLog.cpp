#include "MyLog.h"

MyLog* MyLog::m_Log = NULL;
MyLog::GC MyLog::gc;

MyLog::MyLog()
{
	log4cplus::initialize();

	string propertyPath = "logconfig.properties";
	PropertyConfigurator::doConfigure(LOG4CPLUS_TEXT(propertyPath));
	root_logger = Logger::getInstance(LOG4CPLUS_TEXT("root"));
	console_logger = Logger::getInstance(LOG4CPLUS_TEXT("consle"));
}

MyLog::~MyLog()
{

}

MyLog* MyLog::getInstance()
{
	if (m_Log == NULL)
	{
		m_Log = new MyLog();
	}

	return m_Log;
}