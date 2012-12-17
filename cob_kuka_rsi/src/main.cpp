#include <iostream>
#include "cob_kuka_rsi/RSIConnector.h"

/* Main function */
int main()
{
	std::cout << "About to start... please press ENTER" << std::endl;
	getchar();
	RSIConnector connector(true);
	connector.connect("192.1.10.1", 49150);
	connector.start();

	std::cout << "About to turn ax6... please press ENTER" << std::endl;
	getchar();
	RSIConnector::AxisCorrection axisCor;
	axisCor.dA6 = 0.01f;
	connector.SetAxisCorrection(axisCor);
	
	std::cout << "About to stop... please press ENTER" << std::endl;
	getchar();
	axisCor.dA6 = 0;
	connector.SetAxisCorrection(axisCor);

	std::cout << "About to terminate... please press ENTER" << std::endl;
	getchar();
	connector.stop();
	return 0;
}

