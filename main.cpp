// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <string>

#define HEAD_YAW_MAX 44
#define HEAD_YAW_MIN -44

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

int main(int argc, char *argv[]) 
{
    Network yarp;

    Property params;
    params.fromCommand(argc, argv);

    if (!params.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        return -1;
    }
    
    if (!params.check("repetitions"))
    {
        fprintf(stderr, "Please specify number of repetitions\n");
        fprintf(stderr, "--repetitions num (e.g. 2)\n");
        return -1;
    }
    
    std::string robotName=params.find("robot").asString().c_str();
   // std::string robotName = "icubGazeboSim";
    std::string remotePorts="/";
    remotePorts+=robotName;
    //remotePorts+="/right_arm";
	remotePorts+="/head";

    int numTimes = params.find("repetitions").asInt();
    std::string localPorts="/headMovement_koroibot/client";

    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", localPorts.c_str());   //local port names
    options.put("remote", remotePorts.c_str());         //where we connect to

    // create a device
    PolyDriver robotDevice(options); 
    if (!robotDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    IPositionControl *pos;
    IEncoders *encs;

    bool ok;
    ok = robotDevice.view(pos);
    ok = ok && robotDevice.view(encs);

    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return 0;
    }

    int nj=0;
    pos->getAxes(&nj);
    Vector encoders;
    Vector command;
    Vector tmp;
    encoders.resize(nj);
    tmp.resize(nj);
    command.resize(nj);
    
    int i;
    for (i = 0; i < nj; i++) {
         tmp[i] = 50.0;
    }
    pos->setRefAccelerations(tmp.data());

    for (i = 0; i < nj; i++) {
        tmp[i] = 10.0;
        pos->setRefSpeed(i, tmp[i]);
    }

    //pos->setRefSpeeds(tmp.data()))
    
    //fisrst read all encoders
    //
    printf("waiting for encoders");
    while(!encs->getEncoders(encoders.data()))
    {
        Time::delay(0.1);
        printf(".");
    }
    printf("\n;");
    
    int ctr =0;
    bool done = false;
    while(ctr<numTimes)
    {

      
      printf("Starting headMovement\n");
      command=encoders;
      if(ctr%2 == 0)
        command[2]=HEAD_YAW_MAX;
      else
        command[2]=HEAD_YAW_MIN;
      done = false;
      pos->positionMove(command.data());
      while(!done)
      {
	    pos->checkMotionDone(&done);
	    Time::delay(0.1);
       }
       ctr++;
    }
    
    command[2] = 0;
    pos->positionMove(command.data());
    while(!done)
    {
	  
        pos->checkMotionDone(&done);
	Time::delay(0.1);
    }
	
    robotDevice.close();
   
    return 0;
}
