
#include "MTig.h"


MTig::MTig() {
    iter=0;
    value=false;
    temp_init=1;
    tdata=0;
    min_temp=99;
    max_temp=99;
    nb_sat_OK=0;
    nb_sat=0;
    mtCount=0;
    XsensResultValue res=XRV_ERROR ;
    portCount=0;


}

bool MTig::Selftest(diagnostic_msgs::SelfTest::Request& request, diagnostic_msgs::SelfTest::Response& response) {

    int mask=0x01;
    mtCount=cmt3.getMtCount();
    int status=0;
    diagnostic_updater::DiagnosticStatusWrapper data;

    for (unsigned int k = 0; k < mtCount; k++) {
        status=packet->getStatus();
    }
    if ((mask & status)==mask) {
        value =true;
        data.level=0;
        data.message="The power-up self test completed successfully";
    } else {
        value =false;
        data.level=2;
        data.message="The power-up self test did not complete successfully";
    }
    data.name= "Self Test";
    data.hardware_id=int2str(device_id);


    response.status.push_back(data);
    response.passed=1;

    iter ++;
    std::string nb =int2str(iter);
    response.id=("Call number "+ nb +" to selt test service");


    return value;

}

int MTig::doHardwareScan() {
    List<CmtPortInfo> portInfo;
    portCount = 0;

    ROS_INFO("Scanning for connected Xsens devices...");
    xsens::cmtScanPorts(portInfo);
    portCount = portInfo.length();
    ROS_INFO("done\n");

    if (portCount == 0) {
        ROS_ERROR("No MotionTrackers found\n\n");
        return 0;
    }

    for (int i = 0; i < (int)portCount; i++) {
        ROS_INFO("Using COM port %s at ", portInfo[i].m_portName);

        switch (portInfo[i].m_baudrate) {
        case B9600  :
            ROS_INFO("9k6");
            break;
        case B19200 :
            ROS_INFO("19k2");
            break;
        case B38400 :
            ROS_INFO("38k4");
            break;
        case B57600 :
            ROS_INFO("57k6");
            break;
        case B115200:
            ROS_INFO("115k2");
            break;
        case B230400:
            ROS_INFO("230k4");
            break;
        case B460800:
            ROS_INFO("460k8");
            break;
        case B921600:
            ROS_INFO("921k6");
            break;
        default:
            ROS_INFO("0x%lx", portInfo[i].m_baudrate);
        }
        ROS_INFO(" baud\n\n");
    }

    ROS_INFO("Opening ports...");
    //open the port which the device is connected to and connect at the device's baudrate.
    for (int p = 0; p < (int)portCount; p++) {
        res = cmt3.openPort(portInfo[p].m_portName, portInfo[p].m_baudrate);
        EXIT_ON_ERROR(res,"cmtOpenPort");
    }
    ROS_INFO("done\n\n");

    //get the Mt sensor count.
    ROS_INFO("Retrieving MotionTracker count (excluding attached Xbus Master(s))\n");
    mtCount = cmt3.getMtCount();
    ROS_INFO("MotionTracker count: %d\n\n", mtCount);

    // retrieve the device IDs
    ROS_INFO("Retrieving MotionTrackers device ID(s)\n");
    for (int j = 0; j < mtCount; j++) {
        res = cmt3.getDeviceId((unsigned char)(j+1), deviceIds[j]);
        EXIT_ON_ERROR(res,"getDeviceId");
        ROS_INFO("Device ID at busId %i: %08lx\n\n",j+1,(long) deviceIds[j]);
    }

    return mtCount;
}

void MTig::setUserInputs() {
    mode = CMT_OUTPUTMODE_ORIENT | CMT_OUTPUTMODE_POSITION | CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_VELOCITY |CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_STATUS;


    if ((mode & CMT_OUTPUTMODE_ORIENT) != 0) {
        settings = CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION;

    }
    settings |= CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
}

void MTig::doMtSettings() {
    mtCount = cmt3.getMtCount();

    // set sensor to config sate
    res = cmt3.gotoConfig();
    if (res != XRV_OK) {
        EXIT_ON_ERROR(res,"gotoConfig");
        ROS_ERROR("Go to config state impossible");
    }


    unsigned short sampleFreq;
    sampleFreq = cmt3.getSampleFrequency();

    // set the device output mode for the device(s)
    ROS_INFO("Configuring your mode selection\n");

    for (unsigned int i = 0; i < mtCount; i++) {
        CmtDeviceMode deviceMode(mode, settings, sampleFreq);
        if ((deviceIds[i] & 0xFFF00000) != 0x00500000) {
            // not an MTi-G, remove all GPS related stuff
            deviceMode.m_outputMode &= 0xFF0F;

        }

        res = cmt3.setDeviceMode(deviceMode, true, deviceIds[i]);
        if (res != XRV_OK) {
            ROS_ERROR("Set the device impossible");
            EXIT_ON_ERROR(res,"setDeviceMode");
        }

    }

    // start receiving data
    res = cmt3.gotoMeasurement();
    if (res == XRV_OK) {
        ROS_INFO("MTi -- Setup complete! Initiating data streaming...");
    }

    else {
        ROS_ERROR("Impossible gotoMeasurement");
        EXIT_ON_ERROR(res,"gotoMeasurement");
    }
}

// Converter fonction from integer in hexadecimal to string /////////////////////////////////////////////////////////
std::string MTig::int2str ( int number ) {
    std::ostringstream oss;

    // Works just like cout
    //oss<< number;
    oss << std::hex << number;
    // Return the underlying string
    return oss.str();
}


//Get device id in order to store id in string message

long MTig::getlongid() {


    mtCount = cmt3.getMtCount();
    device_id=0;

    for (unsigned int k = 0; k < mtCount; k++) {
        device_id= (long) deviceIds[k];
    }

    return device_id;

}
