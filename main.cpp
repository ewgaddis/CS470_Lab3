#define _CRT_SECURE_NO_DEPRECATE 1
#include <iostream>
#include <conio.h>

#include "team.h"
#include "gridWindow.h"

using namespace std;

const char *kDefaultServerName = "localhost";
const int kDefaultServerPort = 52573;

int main(int argc, char *argv[])
{
	const char *pcHost;
	int nPort;

	if(argc < 2)
	{
		pcHost = kDefaultServerName;
	}
	else
	{
		pcHost = argv[1];
	}
    if(argc < 3)
	{
		nPort = kDefaultServerPort;
	}
	else
	{
        nPort = atoi(argv[2]);
    }

	cout << "Host: " << pcHost << endl;
	cout << "Port: " << nPort  << endl;

	BZRC MyTeam = BZRC(pcHost, nPort, false);
	if(!MyTeam.GetStatus())
	{
		cout << "Can't connect to BZRC server." << endl;

		while(!_kbhit());
		exit(1);
	}

	// Initialize grid window
	initializeGridWindow();

	// Calling agent code
	world_init(&MyTeam);

	bool exit = false;
	while(!exit)
	{
		robot_pre_update();
		exit = !robot_update();
		robot_post_update();
	}

	shutdownGridWindow();

	MyTeam.Close();

	cout << "Press any key to continue";
	while(!_kbhit());
	return 0;
}