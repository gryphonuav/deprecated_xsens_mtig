#ifndef __MTIG_H
#define __MTIG_H

#include "cmtsrc/cmtdef.h"
#include "cmtsrc/xsens_time.h"
#include "cmtsrc/xsens_list.h"
#include "cmtsrc/cmtscan.h"
#include "cmtsrc/cmt3.h"
#include "cmtsrc/example_linux.h"
#include "cmtsrc/cmt2.h"

#include <iostream>
#include <string>
#include <diagnostic_msgs/SelfTest.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

// this macro tests for an error and exits the program with a message if there was one
#define EXIT_ON_ERROR(res,comment) if (res != XRV_OK) { ROS_INFO("Error %d occurred in " comment ": %s\n",res,xsensResultText(res)); exit(1); }

using namespace std;
using namespace xsens;

	class MTig
	{
		public:
		
		//Methods
		MTig();
		bool Selftest(diagnostic_msgs::SelfTest::Request& request, diagnostic_msgs::SelfTest::Response& response);
		int getit(int &it_init);
		int setit(int &it_init);
		void setUserInputs();
		void doMtSettings(); 
		std::string int2str ( int number );
		long getlongid();
		int doHardwareScan();
		
		//Attributs
		Cmt3 cmt3;
		Cmt2s cmt2;
		CmtOutputMode mode;
		CmtOutputSettings settings;
		unsigned long mtCount;
		CmtDeviceId deviceIds[256];
		XsensResultValue res;
		Packet* packet;
		
		//structs to hold data.
		CmtCalData caldata;
		CmtQuat qat_data;
		CmtEuler euler_data;
		CmtMatrix matrix_data;
		
		
		//Parameters	
		int iter;
		bool value;
		string portname;
		int baudrate;
		int temp_init;
		double tdata;
		double min_temp, max_temp;
		int nb_sat_OK;
		int nb_sat;
		long device_id;
		string frame_id;
		unsigned long portCount;
	
		
	};


#endif




