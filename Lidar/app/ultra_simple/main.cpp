// ConsoleApplication1.cpp : définit le point d'entrée pour l'application console.
//


/*
*  RPLIDAR
*  Ultra Simple Data Grabber Demo App
*
*  Copyright (c) 2009 - 2014 RoboPeak Team
*  http://www.robopeak.com
*  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
*  http://www.slamtec.com
*
*/
/*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*/
#define PI 3.14159265
#define ecart_max 0.02
float dTours = 300;
#include <iostream>
#include <fstream>


#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <list>
#include <unordered_map>
#include <tuple>
#include <chrono>
#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif
#ifdef _WIN32

#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms) {
	while (ms >= 1000) {
		usleep(1000 * 1000);
		ms -= 1000;
	};
	if (ms != 0)
		usleep(ms * 1000);
}
#endif

using namespace rp::standalone::rplidar;

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
	u_result     op_result;
	rplidar_response_device_health_t healthinfo;


	op_result = drv->getHealth(healthinfo);
	if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
		printf("RPLidar health status : %d\n", healthinfo.status);
		if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
			fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
			// enable the following code if you want rplidar to be reboot by software
			// drv->reset();
			return false;
		}
		else {
			return true;
		}

	}
	else {

		fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
		return false;
	}
}

#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
	ctrl_c_pressed = true;
}


bool acceptable(float d1, float d2) {
	return ((d1 > 0) && (d2 > 0) && (abs(d1 - d2) < 100));
}

bool acceptable(int i1, int j1, int i2, int j2) {
	int ecart1 = min(abs(i1 - j1), abs(i1 - j1 - 360));
	int ecart2 = min(abs(i2 - j2), abs(i2 - j2 - 360));
	return ((i1 > 0) && (j1 > 0) && (i2 > 0) && (j2 > 0)&& (ecart1 + ecart2 < 25 + 5*((i1>j1 && i2>j2)|| (i1<j1 && i2<j2))));

}
float scoreQuad(int l1, int l2, int r1, int r2, float d1, float d2) {
	int scoR = min((r2 - r1)*(r2 - r1), min((360 + r1 - r2)*(360 + r1 - r2), (360 + r2 - r1)*(360 + r2 - r1)));
	int scoL = min((l2 - l1)*(l2 - l1), min((360 + l1 - l2)*(360 + l1 - l2), (360 + l2 - l1)*(360 + l2 - l1)));
	float scoD = min((d2 - d1)*(d2 - d1), min((360 + d1 - d2)*(360 + d1 - d2), (360 + d2 - d1)*(360 + d2 - d1)));
	return (scoD + 10*scoR + 10*scoL);
}

int moyenne(int i, int j) {
	int m = (i + j) / 2;
	if (abs(j - i) > 180) {
		return ((m + 180) % 360);
	}
	return m;
}

std::tuple<float, float> pos(std::tuple<int, int, float> t1, std::tuple<int, int, float> t2) {
	int l1 = std::get<0>(t1);
	int r1 = std::get<1>(t1);
	float d1 = std::get<2>(t1);
//	printf("d1 %08.2f \n", d1);
	int m1 = moyenne(l1, r1);
	int l2 = std::get<0>(t2);
	int r2 = std::get<1>(t2);
	float d2 = std::get<2>(t2);
	int m2 = moyenne(l2, r2);
	float y = (dTours*dTours + d1*d1 - d2*d2) / (2 * dTours);
	float x = sqrt(d1*d1 - y*y);
	return std::make_tuple(x, y);
}


// fonction de recherche
std::tuple<int, int, float> search(std::tuple<int, int, float> t, float *table) {
	int l = std::get<0>(t);
	int r = std::get<1>(t);
	float d = std::get<2>(t);
	float score = 1200; //différence quadratique entre les objets
	int jumped = 0,left =0,dMin=10000;
	float ecart = 0;
	bool object = false;
	int l0 = l , r0=r;
	float d0=d;
	// on établit la liste des objets dans le champ et on choisit le meilleur vis à vis d'un écart quadratique (que l'on pourra pondérer du mouvement)
	for (int i = -40; i < 40; i++) {
		int current = moyenne(l,r) + i;
		if (current < 0) { current += 360; }
		if (current > 359) current -= 360;
		if (table[current] == 0) jumped++;
		ecart = abs(table[current] - table[(current + 1)%360]);

		if (table[current] == 0 || ecart > table[current] * ecart_max) {

			if (object) {
				if (score > scoreQuad(left,l,current,r,dMin,d)) {
					score = scoreQuad(left, l, current, r, dMin, d);
					l0 = left;
					r0 = current;
					d0 = dMin;


					//printf("intermédiaire commence %d , termine %d, distance %08.2f score %02.2f \n", l0,r0,d0,score);
				}
				object = false;
				dMin = 10000;
			}
		}
		else {
			jumped = 0;
			if (!object) {
				object = true;
				left = current;

			}
			dMin = min(table[current], dMin);

		}


	}
	return std::make_tuple(l0, r0, d0);
}

auto previous = std::chrono::steady_clock::now();
size_t   count;
const int nTracked = 3;

std::tuple<int, int, float> tracked[nTracked];
float table[360];

int main(int argc, const char * argv[]) {


	const char * opt_com_path = NULL;
	_u32         opt_com_baudrate = 115200;
	u_result     op_result;

	printf("Ultra simple LIDAR data grabber for RPLIDAR.\n",
		"Version: ", RPLIDAR_SDK_VERSION"\n");

	// read serial port from the command line...
	if (argc > 1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3"

										  // read baud rate from the command line if specified...
	if (argc > 2) opt_com_baudrate = strtoul(argv[2], NULL, 10);


	if (!opt_com_path) {
#ifdef _WIN32
		// use default com port
		opt_com_path = "\\\\.\\com18";
#else
		opt_com_path = "/dev/ttyUSB0";
#endif
	}
	std::unordered_map<int, std::tuple<int, int, float>> mem;

	// create the driver instance
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

	if (!drv) {
		fprintf(stderr, "insufficent memory, exit\n");
		exit(-2);
	}


	// make connection...
	if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
		fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
			, opt_com_path);
		goto on_finished;
	}

	rplidar_response_device_info_t devinfo;

	// retrieving the device info
	////////////////////////////////////////
	op_result = drv->getDeviceInfo(devinfo);

	if (IS_FAIL(op_result)) {
		fprintf(stderr, "Error, cannot get device info.\n");
		goto on_finished;
	}

	// print out the device serial number, firmware and hardware version number..
	printf("RPLIDAR S/N: ");
	for (int pos = 0; pos < 16; ++pos) {
		printf("%02X", devinfo.serialnum[pos]);
	}

	printf("\n"
		"Firmware Ver: %d.%02d\n"
		"Hardware Rev: %d\n"
		, devinfo.firmware_version >> 8
		, devinfo.firmware_version & 0xFF
		, (int)devinfo.hardware_version);



	// check health...
	if (!checkRPLIDARHealth(drv)) {
		goto on_finished;
	}

	signal(SIGINT, ctrlc);
	drv->startMotor();

	drv->setMotorPWM(450);

	// start scan...
	drv->startScan();

	// fetch result and print it out...
	//partie perso
	table[360] = { 0 };
	//un premier passage pour initialiser tracked



	rplidar_response_measurement_node_t nodes[360 * 2];
	count = _countof(nodes);

	op_result = drv->grabScanData(nodes, count);

	if (IS_OK(op_result)) {
		drv->ascendScanData(nodes, count);

		for (int pos = 0; pos < (int)count; ++pos) {
			table[static_cast<int> ((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f)] = -min(-nodes[pos].distance_q2 / 4.0f, 0);
		}
	}

	tracked[0] = search(std::make_tuple(270, 277, 450),table);
	tracked[1] = search(std::make_tuple(302, 307, 550), table);
    tracked[2] = search(std::make_tuple(302, 307, 550), table);


	while (1) {


		rplidar_response_measurement_node_t nodes[360 * 2];
		size_t   count = _countof(nodes);

		op_result = drv->grabScanData(nodes, count);

		if (IS_OK(op_result)) {
			drv->ascendScanData(nodes, count);

			for (int pos = 0; pos < (int)count; ++pos) {
				/*printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
			   (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ",
			   (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
			   nodes[pos].distance_q2/4.0f,
			   nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);*/
			   //if (nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT != 0) {
				table[static_cast<int> ((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f)] = -min(-nodes[pos].distance_q2 / 4.0f, 0);

			}
		}
		//traitement
		int n = 1;
		for (int i = 0; i < nTracked; i++) {
			tracked[i] = search(tracked[i], table);
		}
		for (auto t : tracked) {

			//printf("objet %d  commence %d , termine %d, distance %08.2f \n", n, std::get<0>(t), std::get<1>(t), std::get<2>(t));


			n++;
		}
		auto position = pos(tracked[0], tracked[1]);
		printf("position x : %02.2f y : %02.2f \n", std::get<0>(position), std::get<1>(position));
// arret
		if (ctrl_c_pressed) {
			ctrl_c_pressed = false;

			for (auto t : tracked) {

				printf("objet %d  commence %d , termine %d, distance %08.2f \n", n, std::get<0>(t), std::get<1>(t), std::get<2>(t));
				n++;
			}

			signal(SIGINT, ctrlc);



			for (int i = 0; i < 200;i++) {
				signal(SIGINT, ctrlc);
				usleep(10000);
				if (ctrl_c_pressed) {
					drv->stop();
					drv->stopMotor();

					break;
				}
            break;

			}

		}

	}



	// done!
on_finished:
	RPlidarDriver::DisposeDriver(drv);

	return 0;


	// on cherche l'objet

}


