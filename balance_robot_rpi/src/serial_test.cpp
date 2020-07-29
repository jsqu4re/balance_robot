#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <vector>

#include <balance_robot_rpi/json.hpp>

using json = nlohmann::json;

using namespace std;

int main(int argc, char ** argv)
{
	char* levelSensorPort = "/dev/ttyACM0"; //Serial Device Address

	int levelSensor = serialOpen(levelSensorPort, 9600);
	wiringPiSetup();
	serialPuts(levelSensor, "DP"); //Send command to the serial device

	string combined = "";

	while (1)
	{
		char buffer[200];
		ssize_t length = read(levelSensor, &buffer, sizeof(buffer));
		if (length == -1)
		{
			cerr << "Error reading from serial port" << endl;
			break;
		}
		else if (length == 0)
		{
			sleep(0.2);
		}
		else
		{
			buffer[length] = '\0';
			combined += string(buffer);

			if (combined.back() == '\n') {
				auto j = json::parse(combined, nullptr, false);
				if (!j.is_discarded()) {
					try {
						cout << "pitch     :  " << j["P"] << endl;
						cout << "roll      :  " << j["R"] << endl;
						cout << "yaw       :  " << j["Y"] << endl;
						cout << "frequency :  " << j["f"] << endl;
						cout << "---" << endl;
					} catch (const std::exception& e) {
						cout << combined << endl;
					}
				} else {
					cout << combined << endl;
				}
				combined = "";
			}
		}
	}

	return 0;
}