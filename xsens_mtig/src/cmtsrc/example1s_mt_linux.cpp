/*! \file example1s_MT_linux.cpp
    \brief Example file for CMT level 1 serial branch.
    
    This file contains a basic example of the CMT level 1 serial interface.
*/

#include "cmt1.h"
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
int quit = 0;

void doquit(int sig)
{
    // got ctrl-c
    quit = 1;
}

void exitFunc1(void)
{
    printf("Exiting.\n");
    //printw("Press a key to exit.\n");
    //getch();
    //endwin();
}

#define EXIT_ERROR(loc) { printf("Error %d occurred during " loc ": %s\n",serial.getLastResult(), xsensResultText(serial.getLastResult())); exit(-1); }

int main(void)
{
    xsens::Cmt1s serial;
    unsigned char buffer[128] = "\xFA\xFF\x10\x00\xF1"; // this is a gotoMeasurement message
    long readSoFar = 0, index;
    uint32_t readThisTime;
    FILE * file;
    atexit(exitFunc1);
    (void) signal(SIGINT, doquit);
    
    file = fopen("/tmp/log.bin", "wb");
    
	char portname[32];    
    printf("Enter the port name you wish to connect to (default: /dev/ttyUSB0): ");
    scanf("%s", portname);
    
	if (strlen(portname) == 0)
		sprintf(portname, "/dev/ttyUSB0");
		
    if (serial.open(portname, B115200) != XRV_OK) 
        EXIT_ERROR("open");

	serial.getPortName(portname);
    printf("Port %s successfully opened, sending a gotoConfig message.\n", 
        portname);

    serial.flushData();

    if (serial.writeData(5, buffer, NULL) != XRV_OK)
        EXIT_ERROR("write");
    
    printf("gotoMeasurement sent, reading data. Press Control-C to stop."
        "\n-------------------------------------------------------------------------------\n");
    
    
    while (!quit)
    {
        serial.readData(128, buffer, &readThisTime);
        readSoFar += readThisTime;
        index = 0;
        fwrite(buffer, 1, readThisTime, file);
//        while(readThisTime-- > 0)
//            printf("%02X", buffer[index++]);
    }
    
    printf("\n-------------------------------------------------------------------------------\n"
        "Read a total of %ld characters.\n",readSoFar);
    
    if (serial.close() != XRV_OK)
        EXIT_ERROR("close");
    
    printf("Port %d successfully closed.\n", serial.getPortNr());
    fclose(file);
    (void) signal(SIGINT, SIG_DFL);
    return 0; 
}
