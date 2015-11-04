// ARDrone Control in C++
// based on JavaDrone http://code.google.com/p/javadrone/
// and droneController http://dronecontroller.codeplex.com/

#include <commonc++/DatagramSocket.h++>
#include <commonc++/Mutex.h++>
#include <commonc++/Thread.h++>
#include <commonc++/Common.h++>
#include <commonc++/Lock.h++>
#include <commonc++/String.h++>
#include <commonc++/ScopedLock.h++>



#include <string>
#include <sstream>
#include <vector>

#ifndef __APPLE__
#include "malloc.h"    
#endif
#include "stdlib.h"

#include "ARDrone.h"


//////////////////////////////////////////////////
/////////////////////////////////////////////////////////
int nomain(int argc, char** argv)
{
  ARDrone::Controller Controller;
  ARDrone::NavigationData navData;
  try 
  {
    Controller.connectWithDroneAtAddress("192.168.1.1");
    ccxx::Thread::sleep(200);
    ARDrone::NavigationDataReceiver navDataReceiver(&Controller, "192.168.1.1");
    navDataReceiver.start();
    ccxx::Thread::sleep(200);
    
    ARDrone::VideoDataReceiver vdoDataReceiver(&Controller, "192.168.1.1");
    vdoDataReceiver.start();
    ccxx::Thread::sleep(200);
    
    Controller.switchToFrontCamera();
    ARDrone::VideoDecoder::Image image;
    for(int i = 0; i < 1000; i++)
    {
      Controller.sendWatchDogReset();

      ccxx::Thread::sleep(200);
      
      navDataReceiver.copyDataTo(navData);
      std::cout << "->" << navData.orientation.pitch << ','
      << navData.orientation.roll << ','
      << navData.orientation.yaw << std::endl;
      
      vdoDataReceiver.copyDataTo(image);
      
      //std::cout << "w->" << width << ",h->" << height << std::endl;
      
    } 
    navDataReceiver.stop();
    vdoDataReceiver.stop();
  }
  catch(ccxx::Exception& ex) 
  {
    std::cout << ex.what() << std::endl;
  }
  return 0;
}

////////////////////////////////////////////////
#define NAVDATA_DEMO_TAG (0)
#define NAVDATA_TIME_TAG (1)
#define NAVDATA_RAW_MEASURES_TAG (2) 
#define NAVDATA_PHYS_MEASURES_TAG (3) 
#define NAVDATA_GYROS_OFFSETS_TAG (4)
#define NAVDATA_EULER_ANGLES_TAG (5)
#define NAVDATA_REFERENCES_TAG (6)
#define NAVDATA_TRIMS_TAG (7)
#define NAVDATA_RC_REFERENCES_TAG (8)
#define NAVDATA_PWM_TAG (9)
#define NAVDATA_ALTITUDE_TAG (10)
#define NAVDATA_VISION_RAW_TAG (11)
#define NAVDATA_VISION_OF_TAG (12)
#define NAVDATA_VISION_TAG (13)
#define NAVDATA_VISION_PERF_TAG (14)
#define NAVDATA_TRACKERS_SEND_TAG (15)
#define NAVDATA_VISION_DETECT_TAG (16)
#define NAVDATA_WATCHDOG_TAG (17)
#define NAVDATA_ADC_DATA_FRAME_TAG (18)
#define NAVDATA_VIDEO_STREAM_TAG (19)
#define NAVDATA_CKS_TAG (0xFFFF)

/////////////////////////////////////////////

typedef float               float32_t;
typedef double              float64_t;



namespace ARDrone
{
  /////////////////////////////////////////////////////
  Drone::Drone()
  {
    myController = NULL;
    myVideoDataReceiver = NULL;
    myNavigationDataReceiver = NULL;
  }
  
  Drone::~Drone()
  {
    stop();
  }
  
  bool Drone::start(const char* szDroneAddress)
  {
    try 
    {
      myController = new ARDrone::Controller;
      myVideoDataReceiver = new ARDrone::VideoDataReceiver(myController,  szDroneAddress); 
      myNavigationDataReceiver = new ARDrone::NavigationDataReceiver(myController,  szDroneAddress);
      
      
      myController->connectWithDroneAtAddress(szDroneAddress);
      ccxx::Thread::sleep(200);
      
      myNavigationDataReceiver->start();
      ccxx::Thread::sleep(200);
    
      myVideoDataReceiver->start();
      ccxx::Thread::sleep(200);
      return true;
    }
    catch(ccxx::Exception& ex) 
    {
      std::cout << ex.what() << std::endl;
      delete myVideoDataReceiver;
      delete myNavigationDataReceiver;
      delete myController;
      return false;
    }
  }
  
  void Drone::stop()
  {
    if(NULL == myNavigationDataReceiver || NULL == myVideoDataReceiver)
      return;
    try 
    {
      myNavigationDataReceiver->stop();
      myVideoDataReceiver->stop();
    }
    catch(ccxx::Exception& ex) 
    {
      std::cout << ex.what() << std::endl;
    }
    
    if(NULL != myVideoDataReceiver)
      delete myVideoDataReceiver;
    if(NULL != myNavigationDataReceiver)
      delete myNavigationDataReceiver;
    if(NULL != myController)
      delete myController;
    
    myController = NULL;
    myVideoDataReceiver = NULL;
    myNavigationDataReceiver = NULL;
  }
  
  ARDrone::Controller& Drone::controller()
  {
    return *myController;
  }
  
  ARDrone::VideoDataReceiver& Drone::videoDataReceiver()
  {
    return *myVideoDataReceiver;
  }
  
  ARDrone::NavigationDataReceiver& Drone::navigationDataReceiver()
  {
    return *myNavigationDataReceiver;
  }
  ////////////////////////////////////////////////////
  const int kNavigationDataPort = 5554;
  const int kOnBoardVideoPort = 5555;
  const int kATCommandPort = 5556;
  
  
  //NavData offset
  const int kNavigationDataOffsetOfStateData    =  4;
  const int kNavigationDataOffsetOfBatteryData  = 24;
  const int kNavigationDataOffsetOfAltitudeData = 40;
  
  const unsigned int INTERVAL = 100;
  
  ///
  unsigned int myNextATSequence;
  unsigned int myLastATSequence;

  ///
  ATCommand myLastATCommand;

  ///////////////////////////////////////////////////////
  inline int floatToIntegerByteByByte(float f)
  {
    int result;
    ::memcpy(&result, &f, sizeof(int));
    return result;
  }
  ////////////////////////////////////////////////////////
  CommunicationChannel::CommunicationChannel()
  {
    myNextATSequence = 1;
    myLastATSequence = 1;
    myDatagram.init();
  }
  
  CommunicationChannel::~CommunicationChannel()
  {
    disconnectFromDrone();
  }
  
  void CommunicationChannel::connectWithDroneAtAddress(const char* szDroneIpAddress, int iPort)
  {
    ccxx::String strAddr(szDroneIpAddress);
    myDatagram.connect(strAddr, iPort);
    myDatagram.setTimeout(3000);
  }
  
  void CommunicationChannel::disconnectFromDrone()
  {
    if(isConnectedWithDrone())
    {
      myDatagram.shutdown();
    }
  }
  
  void CommunicationChannel::setTimeout(int t)
  {
    myDatagram.setTimeout(t);
  }
  
  bool CommunicationChannel::isConnectedWithDrone()
  {
    return myDatagram.isConnected();
  }
  
  void CommunicationChannel::send(unsigned char* bytes, unsigned int length)
  {
    synchronized(myMutex)
    {
      myDatagram.send(bytes, length);
    }
  }
  
  void CommunicationChannel::receive(unsigned char* bytes, unsigned int& bufferLength)
  {  
    int actualReceivedLength = myDatagram.receive(bytes, bufferLength);
    bufferLength = actualReceivedLength;
  }
  
  void CommunicationChannel::sendAT(const char* szHeader, const char* szDetail, unsigned int mssleep)
  {
    {
      std::stringstream strStm;
      strStm << szHeader << nextATSequence() << szDetail << '\r';
      std::string strATCmd = strStm.str();
      myLastATCommand.strCommandHeader = szHeader;
      myLastATCommand.strCommandData = szDetail;
      //std::cout << "Sending AT command -> " << strATCmd << std::endl;
      myDatagram.send((unsigned char*)strATCmd.c_str(), strATCmd.length());
      if(mssleep > 0)
        ccxx::Thread::sleep(mssleep);
    }
  }
  

  unsigned int CommunicationChannel::nextATSequence()
  {
    ccxx::ScopedLock lock(myMutex);
    return myNextATSequence++;
  }
  
  ARDrone::ATCommand CommunicationChannel::lastATCommand()
  {
    ccxx::ScopedLock lock(myMutex);
    return myLastATCommand;
  }
  
  //////////////////////////////////////////////////
  Controller::Controller()
  {
  }
  
  Controller::~Controller()
  {
  }
  
  void Controller::connectWithDroneAtAddress(const char* szDroneIpAddress)
  {
    if(true == myCommunicationChannel.isConnectedWithDrone())
      return;
    myCommunicationChannel.connectWithDroneAtAddress(szDroneIpAddress, kATCommandPort);
    myCommunicationChannel.sendAT("AT*PMODE=", ",2");
    myCommunicationChannel.sendAT("AT*MISC=", ",2,20,2000,3000");
    myCommunicationChannel.sendAT("AT*REF=", ",290717696");
    myCommunicationChannel.sendAT("AT*COMWDG=", "");
    myCommunicationChannel.sendAT("AT*CONFIG=", ",\"control:altitude_max\",\"1000\""); //altitude max 1m
    myCommunicationChannel.sendAT("AT*CONFIG=", ",\"control:control_level\",\"0\""); //0:BEGINNER, 1:ACE, 2:MAX
    myCommunicationChannel.sendAT("AT*CONFIG=", ",\"general:navdata_demo\",\"TRUE\"");
    myCommunicationChannel.sendAT("AT*CONFIG=", ",\"general:video_enable\",\"TRUE\"");
    disableAdaptiveVideo();
    //myCommunicationChannel.sendAT("AT*CONFIG=", ",\"network:owner_mac\",\"00:18:DE:9D:E9:5D\""); //my PC
    //myCommunicationChannel.sendAT("AT*CONFIG=", ",\"network:owner_mac\",\"00:23:CD:5D:92:37\""); //AP
    myCommunicationChannel.sendAT("AT*CONFIG=", ",\"pic:ultrasound_freq\",\"8\"");
    myCommunicationChannel.sendAT("AT*FTRIM=", ""); //flat trim
    
    myCommunicationChannel.sendAT("AT*REF=", ",290717696");
    sendControlParameters(0, 0, 0, 0, 0);
    myCommunicationChannel.sendAT("AT*REF=", ",290717696");
    myCommunicationChannel.sendAT("AT*REF=", ",290717696");
  }
  
  void Controller::sendWatchDogReset()
  {
    myCommunicationChannel.sendAT("AT*COMWDG=", "");
  }
  
  void Controller::requestNavigationData()
  {
    myCommunicationChannel.sendAT("AT*CONFIG=", ",\"general:navdata_demo\",\"TRUE\"", 20);
  }
  
  void Controller::requestVideoData()
  {
    myCommunicationChannel.sendAT("AT*CONFIG=", ",\"general:video_enable\",\"TRUE\"", 10);
  }
  
  void Controller::sendFlatTrim()
  {
    myCommunicationChannel.sendAT("AT*FTRIM=", "");
  }
  
  void Controller::switchToFrontCamera()
  {
    myCommunicationChannel.sendAT("AT*ZAP=", "0");
  }
  
  void Controller::switchToDownCamera()
  {
    myCommunicationChannel.sendAT("AT*ZAP=", "2");
  }
  
  void Controller::takeOff()
  {
    myCommunicationChannel.sendAT("AT*REF=", ",290718208");
  }
  
  void Controller::land()
  {
    myCommunicationChannel.sendAT("AT*REF=", ",290717696");
  }
  
  void Controller::sendEmergencyShutdown()
  {
    myCommunicationChannel.sendAT("AT*REF=", ",290717952"); //toggle Emergency
  }
  
  void Controller::hover()
  {
    sendControlParameters(0, 0, 0, 0, 0);
  }
  
  void Controller::sendControlParameters(int enable, float pitch, float roll, float yaw, float gaz)
  {
    std::stringstream strStm;
    strStm << "," << enable << ','
           << floatToIntegerByteByByte(roll) << ','
           << floatToIntegerByteByByte(pitch) << ','
           << floatToIntegerByteByByte(gaz) << ','
           << floatToIntegerByteByByte(yaw);
    
    myCommunicationChannel.sendAT("AT*PCMD=", strStm.str().c_str());
  }
  
  void Controller::sendLastCommand()
  {
    ARDrone::ATCommand cmd = myCommunicationChannel.lastATCommand();
    myCommunicationChannel.sendAT(cmd.strCommandHeader.c_str(), cmd.strCommandData.c_str());
  }
  
  void Controller::disableAdaptiveVideo()
  {
    //ardrone_at_set_toy_configuration("video:bitrate_ctrl_mode","0")
    //"AT*CONFIG=%d,\"%s\",\"%s\"\r",
    
    myCommunicationChannel.sendAT("AT*CONFIG=", "video:bitrate_ctrl_mode,0");
  }
  
  /* ARDroneME --- Java (J2ME) based AR.Drone Controller
   Author: MAPGPS at
   http://www.ardrone-flyers.com/forum/viewforum.php?f=8
   http://www.rcgroups.com/forums/showthread.php?t=1401935
   https://projects.ardrone.org/projects/ardrone-api/boards
   http://www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1025
   http://bbs.5imx.com/bbs/viewthread.php?tid=415063
   Initial: 2011.03.13
   
   
   ########## Keyboad Layout ############
   Takeoff/Landing: a toggle button (or MediaPlayer button)
   Emergency: "E" button (or Camera button), only effective after Landing button pressed first)
   Hovering: when the Arrow button loosed
   Speed(%) slider: change rudder rate in range of 0%~90%
   Arrow Keys and 2 Soft-Joysticks on the touch screen are linked.
   
   Arrow Keys:
   Go Up
   ^
   |
   Go Left <---+---> Go Right
   |
   v
   Go Down
   
   Arrow Keys with central button pressed down (Shift):
   Go Forward
   ^
   |
   Rotate Left <--- ---> Rotate Right
   |
   v
   Go Backward
   
   UI_BIT:
   00010001010101000000000000000000
   |   | | | |        || | ||||+--0: Button turn to left
   |   | | | |        || | |||+---1: Button altitude down (ah - ab)
   |   | | | |        || | ||+----2: Button turn to right
   |   | | | |        || | |+-----3: Button altitude up (ah - ab)
   |   | | | |        || | +------4: Button - z-axis (r1 - l1)
   |   | | | |        || +--------6: Button + z-axis (r1 - l1)
   |   | | | |        |+----------8: Button emergency reset all
   |   | | | |        +-----------9: Button Takeoff / Landing
   |   | | | +-------------------18: y-axis trim +1 (Trim increase at +/- 1??/s)
   |   | | +---------------------20: x-axis trim +1 (Trim increase at +/- 1??/s)
   |   | +-----------------------22: z-axis trim +1 (Trim increase at +/- 1??/s)
   |   +-------------------------24: x-axis +1
   +-----------------------------28: y-axis +1
   
   AT*REF=<sequence>,<UI>
   AT*PCMD=<sequence>,<enable>,<roll>,<pitch>,<gaz>,<yaw>
   (float)0.05 = (int)1028443341           (float)-0.05 = (int)-1119040307
   (float)0.1  = (int)1036831949           (float)-0.1  = (int)-1110651699
   (float)0.2  = (int)1045220557           (float)-0.2  = (int)-1102263091
   (float)0.5  = (int)1056964608           (float)-0.5  = (int)-1090519040
   AT*ANIM=<sequence>,<animation>,<duration>
   AT*CONFIG=<sequence>,\"<name>\",\"<value>\"
   
   ########## AT Commands ############
   altitude max2m: AT*CONFIG=1,\"control:altitude_max\",\"2000\"   //10000=unlimited
   Takeoff:        AT*REF=1,290718208
   Landing:        AT*REF=1,290717696
   Hovering:       AT*PCMD=1,1,0,0,0,0
   gaz 0.1:        AT*PCMD=1,1,0,0,1036831949,0
   gaz -0.1:       AT*PCMD=1,1,0,0,-1110651699,0
   roll 0.1:       AT*PCMD=1,1,1036831949,0,0,0
   roll -0.1:      AT*PCMD=1,1,-1110651699,0,0,0
   yaw 0.1:        AT*PCMD=1,1,0,0,0,1036831949
   yaw -0.1:       AT*PCMD=1,1,0,0,0,-1110651699
   pitch 0.1:      AT*PCMD=1,1,0,1036831949,0,0
   pitch -0.1:     AT*PCMD=1,1,0,-1110651699,0,0
   pitch -30 deg:  AT*ANIM=1,0,1000
   pitch 30 deg:   AT*ANIM=1,1,1000
   Emergency       AT*REF=1,290717952
   Flat Trim:      AT*FTRIM=1
   */
  
  ///////////////////////////////////////////////
  NavigationDataReceiver::NavigationDataReceiver(ARDrone::Controller* pController, const char* szDroneIpAddress)
  {
    myDroneAddress = szDroneIpAddress;
    myController = pController;
  }
  
  NavigationDataReceiver::~NavigationDataReceiver() throw ()
  {
    if(true == isRunning())
    {
      try
      {
        stop();
        join();
        myCommunicationChannel.disconnectFromDrone();
      }
      catch (ccxx::Exception& ex) 
      {
        std::cout << ex.what() << std::endl;
      }
    }
  }
  
 
  void NavigationDataReceiver::run()
  {
    std::cout << "NavigationDataReceiver started\n";
    try 
    {
      myCommunicationChannel.connectWithDroneAtAddress(myDroneAddress.c_str(), kNavigationDataPort);
      myCommunicationChannel.setTimeout(3000);
      
      unsigned char trigger[4] = {0x01, 0x00, 0x00, 0x00};
      myCommunicationChannel.send(trigger, 4);
      
      
      
      unsigned char navDataDemo[10240];
      unsigned int navDataLength = 10240;
      while(false == testCancel())
      {
        try 
        {
          synchronized(myMutex)
          {
            myController->requestNavigationData();
            myCommunicationChannel.receive(navDataDemo, navDataLength);
            
            MemoryLibrary::Buffer navDataBuffer(navDataDemo, navDataLength);
            parse(navDataBuffer);
          }
        }
        catch (ccxx::TimeoutException& timeoutEx) 
        {
          std::cout << "NavigationDataReceiver TIMEOUT exception thrown.." << timeoutEx.what() << std::endl;
        }
        catch (ccxx::Exception& ex) 
        {
          std::cout << "NavigationDataReceiver exception thrown.." << ex.what() << std::endl;
        }
      }//while
    }
    catch (ccxx::Exception& ex) 
    {
      std::cout << "NavigationDataReceiver exception thrown.." << ex.what() << std::endl;
    }
    
    std::cout << "NavigationDataReceiver stopped\n";
  }
  ////////
  bool NavigationDataReceiver::parse(MemoryLibrary::Buffer& buffer)
  {
    int offset = 0;
    int header = buffer.MakeValueFromOffset<int32_t>(offset);
    if(header != 0x55667788)
    {
      std::cout << "NavigationDataReceiver FAIL, because the header != 0x55667788\n";
      return false;
    }
    
    offset += 4;
    int state = buffer.MakeValueFromOffset<int32_t>(offset);
    parseState(state);
    
    offset += 4;
    myNavData.sequence = buffer.MakeValueFromOffset<int32_t>(offset);
    
    offset += 4;
    // int vision_tag;
    
    offset += 4;
    while(offset < buffer.Size())
    {
      int option_tag = (int)buffer.MakeValueFromOffset<unsigned short>(offset);
      offset += 2;
      int option_len = (int)buffer.MakeValueFromOffset<unsigned short>(offset);
      offset += 2;
      
      if(option_len == 0)
      {
        std::cout << "NavigationDataReceiver FAIL, option_len == 0\n";
        return false;
      }
      
      switch(option_tag)
      {
        case NAVDATA_DEMO_TAG: parseNavigation(buffer, offset); break;
        case NAVDATA_CKS_TAG:  break;
        case NAVDATA_VISION_DETECT_TAG: parseVision(buffer, offset); break;
      }
      
      offset = offset + option_len - 4;
    } //while
    
    return true;
  }
  
  bool NavigationDataReceiver::parseState(int state)
  {
    myNavData.flying = (state & 1) != 0;
    myNavData.videoEnabled = (state & (1 << 1)) != 0;
    myNavData.visionEnabled = (state & (1 << 2)) != 0;
    myNavData.controlAlgorithm = (state & (1 << 3)) != 0 ? NavigationData::eAugularSpeedControl : NavigationData::eEulerAnglesControl;
    myNavData.altitudeControlActive = (state & (1 << 4)) != 0;
    myNavData.userFeedbackOn = (state & (1 << 5)) != 0;
    myNavData.controlReceived = (state & (1 << 6)) != 0;
    myNavData.trimReceived = (state & (1 << 7)) != 0;
    myNavData.trimRunning = (state & (1 << 8)) != 0;
    myNavData.trimSucceeded = (state & (1 << 9)) != 0;
    myNavData.navDataDemoOnly = (state & (1 << 10)) != 0;
    myNavData.navDataBootstrap = (state & (1 << 11)) != 0;
    myNavData.motorsDown = (state & (1 << 12)) != 0;
    myNavData.gyrometersDown = (state & (1 << 14)) != 0;
    myNavData.batteryTooLow = (state & (1 << 15)) != 0;
    myNavData.batteryTooHigh = (state & (1 << 16)) != 0;
    myNavData.timerElapsed = (state & (1 << 17)) != 0;
    myNavData.notEnoughPower = (state & (1 << 18)) != 0;
    myNavData.angelsOutOufRange = (state & (1 << 19)) != 0;
    myNavData.tooMuchWind = (state & (1 << 20)) != 0;
    myNavData.ultrasonicSensorDeaf = (state & (1 << 21)) != 0;
    myNavData.cutoutSystemDetected = (state & (1 << 22)) != 0;
    myNavData.PICVersionNumberOK = (state & (1 << 23)) != 0;
    myNavData.ATCodedThreadOn = (state & (1 << 24)) != 0;
    myNavData.navDataThreadOn = (state & (1 << 25)) != 0;
    myNavData.videoThreadOn = (state & (1 << 26)) != 0;
    myNavData.acquisitionThreadOn = (state & (1 << 27)) != 0;
    myNavData.controlWatchdogDelayed = (state & (1 << 28)) != 0;
    myNavData.ADCWatchdogDelayed = (state & (1 << 29)) != 0;
    myNavData.communicationProblemOccurred = (state & (1 << 30)) != 0;
    myNavData.emergency = (state & (1 << 31)) != 0;
    return true;
  }
  
  bool NavigationDataReceiver::parseNavigation(MemoryLibrary::Buffer& buffer, int offset)
  {
    int temp = buffer.MakeValueFromOffset<int32_t>(offset);
    myNavData.controlState = (NavigationData::eControlState)(temp >> 16);
    offset += 4;
    myNavData.batteryLevel = buffer.MakeValueFromOffset<int32_t>(offset);
    offset += 4;
    myNavData.orientation.pitch = buffer.MakeValueFromOffset<float32_t>(offset) / 1000.0f;
    offset += 4;
    myNavData.orientation.roll = buffer.MakeValueFromOffset<float32_t>(offset) / 1000.0f;
    offset += 4;
    myNavData.orientation.yaw = buffer.MakeValueFromOffset<float32_t>(offset) / 1000.0f;
    offset += 4;
    myNavData.altitude = (float)buffer.MakeValueFromOffset<int32_t>(offset) / 1000.0f;
    offset += 4;
    myNavData.speed.vx = buffer.MakeValueFromOffset<float32_t>(offset);
    offset += 4;
    myNavData.speed.vy = buffer.MakeValueFromOffset<float32_t>(offset);
    offset += 4;
    myNavData.speed.vz = buffer.MakeValueFromOffset<float32_t>(offset);
    offset += 4;
    
    return true;
  }
  
  bool NavigationDataReceiver::parseVision(MemoryLibrary::Buffer& buffer, int offset)
  {
    int nbDetected = buffer.MakeValueFromOffset<int32_t>(offset);
    offset += 4;
    
    if(0 == nbDetected) // not detecting anything
      return true;
    myNavData.visionTagVector.resize(0);
    for(int i = 0; i < nbDetected; i++)
    {
      ARDrone::NavigationData::VisionTag visionTag;
      
      visionTag.type = buffer.MakeValueFromOffset<int32_t>(offset + 4 * i);
      visionTag.x = buffer.MakeValueFromOffset<int32_t>(offset + 4 * i + 1 * nbDetected * 4);
      visionTag.y = buffer.MakeValueFromOffset<int32_t>(offset + 4 * i + 2 * nbDetected * 4);
      visionTag.width = buffer.MakeValueFromOffset<int32_t>(offset + 4 * i + 3 * nbDetected * 4);
      visionTag.height = buffer.MakeValueFromOffset<int32_t>(offset + 4 * i + 4 * nbDetected * 4);
      visionTag.distance = buffer.MakeValueFromOffset<int32_t>(offset + 4 * i + 5 * nbDetected * 4);
      
      myNavData.visionTagVector.push_back(visionTag);
    }
    return true;
  }
  
  void NavigationDataReceiver::copyDataTo(ARDrone::NavigationData& data)
  {
    synchronized(myMutex)
    {
      //::memcpy(&data, &myNavData, sizeof(ARDrone::NavigationData));
      data = myNavData;
    }
  }
  
  ////////////////////////////////////////////////////////////////////////
  VideoDataReceiver::VideoDataReceiver(ARDrone::Controller* pController, const char* szDroneIpAddress)
  {
    myDroneAddress = szDroneIpAddress;
    myController = pController;
  }
  
  VideoDataReceiver::~VideoDataReceiver() throw ()
  {
    if(true == isRunning())
    {
      try
      {
        stop();
        join();
        myCommunicationChannel.disconnectFromDrone();
      }
      catch (ccxx::Exception& ex) 
      {
        std::cout << ex.what() << std::endl;
      }
    }
  }
  
  void VideoDataReceiver::run()
  {
    std::cout << "VideoDataReceiver started\n";
    try 
    {
      myCommunicationChannel.connectWithDroneAtAddress(myDroneAddress.c_str(), kOnBoardVideoPort);
      myCommunicationChannel.setTimeout(3000);
      myController->disableAdaptiveVideo();
      unsigned char trigger[4] = {0x01, 0x00, 0x00, 0x00};
      myCommunicationChannel.send(trigger, 4);
      
      myController->requestVideoData();
      while(false == testCancel())
      {
        try 
        {
          
          synchronized(myMutex)
          {
            videoDataLength = 921600;
            myCommunicationChannel.receive(myVideoData, videoDataLength);
            //::printf("vd length--> %d\n", videoDataLength);
            //::Thread::sleep(20);
          }
        }
        catch (ccxx::TimeoutException& timeoutEx) 
        {
          std::cout << "VideoDataReceiver TIMEOUT exception thrown.." << timeoutEx.what() << std::endl;
        }
        catch (ccxx::Exception& ex) 
        {
          std::cout << "VideoDataReceiver exception thrown.." << ex.what() << std::endl;
        }
      }//while
    }
    catch (ccxx::Exception& ex) 
    {
      std::cout << "VideoDataReceiver exception thrown.." << ex.what() << std::endl;
    }
    
    std::cout << "VideoDataReceiver stopped\n";
  }
  
  void VideoDataReceiver::copyDataTo(ARDrone::VideoDecoder::Image& resultImage)
  {
    synchronized(myMutex)
    {
      ARDrone::VideoDecoder::decodeImage(myVideoData, videoDataLength, resultImage);
      //::printf("%d, %d\n", resultImage.width, resultImage.height);
    }
  }
  
  ////////////////////////////////////////////////////////
  // VideoDecoder starts here
  ////////////////////////////////////////////////////////
  namespace VideoDecoder
  {
    struct RGB24BitsColor 
    {
      unsigned char red, green, blue;
    };
    
    RGB24BitsColor* RGB24OutputPixelData = NULL;
    
    const int kPictureFormatCIF = 1;
    const int kPictureFormatVGA = 2;
    
    const int CONST_BlockWidth = 8;
    const int CONST_BlockSize = 64;
    
    const int CONST_WidthCif = 88;
    const int CONST_HeightCif = 72;
    
    const int CONST_WidthVga = 160;
    const int CONST_HeightVga = 120;
    
    const int CONST_TableQuantization = 31;
    
    const int FIX_0_298631336 = 2446;
    const int FIX_0_390180644 = 3196;
    const int FIX_0_541196100 = 4433;
    const int FIX_0_765366865 = 6270;
    const int FIX_0_899976223 = 7373;
    const int FIX_1_175875602 = 9633;
    const int FIX_1_501321110 = 12299;
    const int FIX_1_847759065 = 15137;
    const int FIX_1_961570560 = 16069;
    const int FIX_2_053119869 = 16819;
    const int FIX_2_562915447 = 20995;
    const int FIX_3_072711026 = 25172;
    
    const int CONST_BITS = 13;
    const int PASS1_BITS = 1;
    int F1 = CONST_BITS - PASS1_BITS - 1;
    int F2 = CONST_BITS - PASS1_BITS;
    int F3 = CONST_BITS + PASS1_BITS + 3;
    
    
    
    short dataBlockBuffer[64]; 
    
    static short zigZagPositions[] =   
    {
      0,  1,  8, 16,  9,  2,  3, 10,
      17, 24, 32, 25, 18, 11,  4,  5,
      12, 19, 26, 33, 40, 48, 41, 34,
      27, 20, 13,  6,  7, 14, 21, 28,
      35, 42, 49, 56, 57, 50, 43, 36,
      29, 22, 15, 23, 30, 37, 44, 51,
      58, 59, 52, 45, 38, 31, 39, 46,
      53, 60, 61, 54, 47, 55, 62, 63,
    };
    
    static short allzeros[] =
    {
      0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0,
    };
    
    //Cfr. Handbook of Data Compression - Page 529
    //David Salomon
    //Giovanni Motta
    
    static short quantizerValues[] = 
    {  
      3,  5,  7,  9, 11, 13, 15, 17,
      5,  7,  9, 11, 13, 15, 17, 19,
      7,  9, 11, 13, 15, 17, 19, 21,
      9, 11, 13, 15, 17, 19, 21, 23,
      11, 13, 15, 17, 19, 21, 23, 25,
      13, 15, 17, 19, 21, 23, 25, 27,
      15, 17, 19, 21, 23, 25, 27, 29,
      17, 19, 21, 23, 25, 27, 29, 31
    };
    
    static unsigned char clzlut[] = 
    { 
      8,7,6,6,5,5,5,5, 
      4,4,4,4,4,4,4,4, 
      3,3,3,3,3,3,3,3, 
      3,3,3,3,3,3,3,3, 
      2,2,2,2,2,2,2,2, 
      2,2,2,2,2,2,2,2, 
      2,2,2,2,2,2,2,2, 
      2,2,2,2,2,2,2,2, 
      1,1,1,1,1,1,1,1, 
      1,1,1,1,1,1,1,1, 
      1,1,1,1,1,1,1,1, 
      1,1,1,1,1,1,1,1, 
      1,1,1,1,1,1,1,1, 
      1,1,1,1,1,1,1,1, 
      1,1,1,1,1,1,1,1, 
      1,1,1,1,1,1,1,1, 
      0,0,0,0,0,0,0,0, 
      0,0,0,0,0,0,0,0, 
      0,0,0,0,0,0,0,0, 
      0,0,0,0,0,0,0,0, 
      0,0,0,0,0,0,0,0, 
      0,0,0,0,0,0,0,0, 
      0,0,0,0,0,0,0,0, 
      0,0,0,0,0,0,0,0, 
      0,0,0,0,0,0,0,0, 
      0,0,0,0,0,0,0,0, 
      0,0,0,0,0,0,0,0, 
      0,0,0,0,0,0,0,0, 
      0,0,0,0,0,0,0,0, 
      0,0,0,0,0,0,0,0, 
      0,0,0,0,0,0,0,0, 
      0,0,0,0,0,0,0,0 
    };
    
    unsigned int StreamField;
    int StreamFieldBitIndex;
    int StreamIndex;
    
    bool PictureComplete;
    
    int PictureFormat;
    int Resolution;
    int PictureType;
    int QuantizerMode;
    int FrameIndex;
    
    int SliceCount;
    int SliceIndex;
    
    int BlockCount;
    
    int Width = -1;
    int Height = -1;
    
    int PixelRowSize;
    
    unsigned char* ImageStream;
    unsigned int ImageStreamLength = 0;
    
    std::vector<unsigned short> PixelData;
    
    struct MacroBlock
    {
      short DataBlocks[8][64]; 
    }; // MacroBlock
    
    struct ImageSlice
    {
      std::vector< ARDrone::VideoDecoder::MacroBlock > MacroBlocks;
      
      void setSize(int macroBlockCount)
      {
        MacroBlocks.resize(macroBlockCount);
      }
      
    }; //ImageSlice
    
    ////////////////////////////////////////////
    ARDrone::VideoDecoder::ImageSlice ImageSlice;
    ////////////////////////////////////////////
    
    
    void InverseTransform(int macroBlockIndex, int dataBlockIndex)
    {
      int workSpace[64];
      short data[64];
      
      int z1, z2, z3, z4, z5;
      int tmp0, tmp1, tmp2, tmp3;
      int tmp10, tmp11, tmp12, tmp13;
      
      int pointer = 0;
      
      for (int index = 8; index > 0; index--)
      {
        if (dataBlockBuffer[pointer + 8] == 0 &&
            dataBlockBuffer[pointer + 16] == 0 &&
            dataBlockBuffer[pointer + 24] == 0 &&
            dataBlockBuffer[pointer + 32] == 0 &&
            dataBlockBuffer[pointer + 40] == 0 &&
            dataBlockBuffer[pointer + 48] == 0 &&
            dataBlockBuffer[pointer + 56] == 0)
        {
          int dcValue = dataBlockBuffer[pointer] << PASS1_BITS;
          
          workSpace[pointer + 0] = dcValue;
          workSpace[pointer + 8] = dcValue;
          workSpace[pointer + 16] = dcValue;
          workSpace[pointer + 24] = dcValue;
          workSpace[pointer + 32] = dcValue;
          workSpace[pointer + 40] = dcValue;
          workSpace[pointer + 48] = dcValue;
          workSpace[pointer + 56] = dcValue;
          
          pointer++;
          continue;
        }
        
        z2 = dataBlockBuffer[pointer + 16];
        z3 = dataBlockBuffer[pointer + 48];
        
        z1 = (z2 + z3) * FIX_0_541196100;
        tmp2 = z1 + z3 * -FIX_1_847759065;
        tmp3 = z1 + z2 * FIX_0_765366865;
        
        z2 = dataBlockBuffer[pointer];
        z3 = dataBlockBuffer[pointer + 32];
        
        tmp0 = (z2 + z3) << CONST_BITS;
        tmp1 = (z2 - z3) << CONST_BITS;
        
        tmp10 = tmp0 + tmp3;
        tmp13 = tmp0 - tmp3;
        tmp11 = tmp1 + tmp2;
        tmp12 = tmp1 - tmp2;
        
        tmp0 = dataBlockBuffer[pointer + 56];
        tmp1 = dataBlockBuffer[pointer + 40];
        tmp2 = dataBlockBuffer[pointer + 24];
        tmp3 = dataBlockBuffer[pointer + 8];
        
        z1 = tmp0 + tmp3;
        z2 = tmp1 + tmp2;
        z3 = tmp0 + tmp2;
        z4 = tmp1 + tmp3;
        z5 = (z3 + z4) * FIX_1_175875602;
        
        tmp0 = tmp0 * FIX_0_298631336;
        tmp1 = tmp1 * FIX_2_053119869;
        tmp2 = tmp2 * FIX_3_072711026;
        tmp3 = tmp3 * FIX_1_501321110;
        z1 = z1 * -FIX_0_899976223;
        z2 = z2 * -FIX_2_562915447;
        z3 = z3 * -FIX_1_961570560;
        z4 = z4 * -FIX_0_390180644;
        
        z3 += z5;
        z4 += z5;
        
        tmp0 += z1 + z3;
        tmp1 += z2 + z4;
        tmp2 += z2 + z3;
        tmp3 += z1 + z4;
        
        workSpace[pointer + 0] = ((tmp10 + tmp3 + (1 << F1)) >> F2);
        workSpace[pointer + 56] = ((tmp10 - tmp3 + (1 << F1)) >> F2);
        workSpace[pointer + 8] = ((tmp11 + tmp2 + (1 << F1)) >> F2);
        workSpace[pointer + 48] = ((tmp11 - tmp2 + (1 << F1)) >> F2);
        workSpace[pointer + 16] = ((tmp12 + tmp1 + (1 << F1)) >> F2);
        workSpace[pointer + 40] = ((tmp12 - tmp1 + (1 << F1)) >> F2);
        workSpace[pointer + 24] = ((tmp13 + tmp0 + (1 << F1)) >> F2);
        workSpace[pointer + 32] = ((tmp13 - tmp0 + (1 << F1)) >> F2);
        
        pointer++;
      }
      
      pointer = 0;
      
      for (int index = 0; index < 8; index++)
      {
        z2 = workSpace[pointer + 2];
        z3 = workSpace[pointer + 6];
        
        z1 = (z2 + z3) * FIX_0_541196100;
        tmp2 = z1 + z3 * -FIX_1_847759065;
        tmp3 = z1 + z2 * FIX_0_765366865;
        
        tmp0 = (workSpace[pointer + 0] + workSpace[pointer + 4]) << CONST_BITS;
        tmp1 = (workSpace[pointer + 0] - workSpace[pointer + 4]) << CONST_BITS;
        
        tmp10 = tmp0 + tmp3;
        tmp13 = tmp0 - tmp3;
        tmp11 = tmp1 + tmp2;
        tmp12 = tmp1 - tmp2;
        
        tmp0 = workSpace[pointer + 7];
        tmp1 = workSpace[pointer + 5];
        tmp2 = workSpace[pointer + 3];
        tmp3 = workSpace[pointer + 1];
        
        z1 = tmp0 + tmp3;
        z2 = tmp1 + tmp2;
        z3 = tmp0 + tmp2;
        z4 = tmp1 + tmp3;
        
        z5 = (z3 + z4) * FIX_1_175875602;
        
        tmp0 = tmp0 * FIX_0_298631336;
        tmp1 = tmp1 * FIX_2_053119869;
        tmp2 = tmp2 * FIX_3_072711026;
        tmp3 = tmp3 * FIX_1_501321110;
        z1 = z1 * -FIX_0_899976223;
        z2 = z2 * -FIX_2_562915447;
        z3 = z3 * -FIX_1_961570560;
        z4 = z4 * -FIX_0_390180644;
        
        z3 += z5;
        z4 += z5;
        
        tmp0 += z1 + z3;
        tmp1 += z2 + z4;
        tmp2 += z2 + z3;
        tmp3 += z1 + z4;
        
        data[pointer + 0] = (short)((tmp10 + tmp3) >> F3);
        data[pointer + 7] = (short)((tmp10 - tmp3) >> F3);
        data[pointer + 1] = (short)((tmp11 + tmp2) >> F3);
        data[pointer + 6] = (short)((tmp11 - tmp2) >> F3);
        data[pointer + 2] = (short)((tmp12 + tmp1) >> F3);
        data[pointer + 5] = (short)((tmp12 - tmp1) >> F3);
        data[pointer + 3] = (short)((tmp13 + tmp0) >> F3);
        data[pointer + 4] = (short)((tmp13 - tmp0) >> F3);
        
        pointer += 8;
      }
      
      
      {
        short* source = data;
        short* destination = ImageSlice.MacroBlocks[macroBlockIndex].DataBlocks[dataBlockIndex];
        {
          //IntPtr sourcePtr = (IntPtr)source;
          //IntPtr destinationPtr = (IntPtr)destination;
          //CopyMemory(destinationPtr, sourcePtr, data.Length * 2);
          ::memcpy(destination, source, 64 * sizeof(short));
        }
      }
      
    } // inverseTransform
    
    ///////////////////////////////////////////
    int CountLeadingZeros(unsigned int value)
    {
      int accum = 0;
      
      accum += clzlut[value >> 24];
      accum += (accum == 8) ? clzlut[(value >> 16) & 0xFF] : 0;
      accum += (accum == 16) ? clzlut[(value >> 8) & 0xFF] : 0;
      accum += (accum == 24) ? clzlut[value & 0xFF] : 0;
      
      return accum;
    } //CountLeadingZeros
    
    ///////////////////////////////////////////
    unsigned short MakeRgb(int r, int g, int b)
    {
      return (unsigned short)((r << 11) | (g << 5) | b);
    }
    
    ///////////////////////////////////////////
    int Saturate5(int x)
    {
      if (x < 0)
      {
        x = 0;
      }
      
      x >>= 11;
      
      return (x > 0x1F) ? 0x1F : x;
    }
    
    int Saturate6(int x)
    {
      if (x < 0)
      {
        x = 0;
      }
      
      x >>= 10;
      
      return x > 0x3F ? 0x3F : x;
    }
    /////////////////////////////////////////////
    void ComposeImageSlice()
    {
      int u, ug, ub;
      int v, vg, vr;
      int r, g, b;
      
      int lumaElementIndex1 = 0;
      int lumaElementIndex2 = 0;
      int chromaOffset = 0;
      
      int dataIndex1 = 0;
      int dataIndex2 = 0;
      
      int lumaElementValue1 = 0;
      int lumaElementValue2 = 0;
      int chromaBlueValue = 0;
      int chromaRedValue = 0;
      
      int cromaQuadrantOffsets[] ={ 0, 4, 32, 36 };
      int pixelDataQuadrantOffsets[] = { 0, CONST_BlockWidth, Width * CONST_BlockWidth, (Width * CONST_BlockWidth) + CONST_BlockWidth };
      
      int imageDataOffset = (SliceIndex - 1) * Width * 16;
      
      //foreach (MacroBlock macroBlock in ImageSlice.MacroBlocks)
      for(int i = 0; i < ImageSlice.MacroBlocks.size(); i++)
      {
        ARDrone::VideoDecoder::MacroBlock macroBlock = ImageSlice.MacroBlocks.at(i);
        for (int verticalStep = 0; verticalStep < CONST_BlockWidth / 2; verticalStep++)
        {
          chromaOffset = verticalStep * CONST_BlockWidth;
          lumaElementIndex1 = verticalStep * CONST_BlockWidth * 2;
          lumaElementIndex2 = lumaElementIndex1 + CONST_BlockWidth;
          
          dataIndex1 = imageDataOffset + (2 * verticalStep * Width);
          dataIndex2 = dataIndex1 + Width;
          
          for (int horizontalStep = 0; horizontalStep < CONST_BlockWidth / 2; horizontalStep++)
          {
            for (int quadrant = 0; quadrant < 4; quadrant++)
            {
              int chromaIndex = chromaOffset + cromaQuadrantOffsets[quadrant] + horizontalStep;
              chromaBlueValue = macroBlock.DataBlocks[4][chromaIndex];
              chromaRedValue = macroBlock.DataBlocks[5][chromaIndex];
              
              u = chromaBlueValue - 128;
              ug = 88 * u;
              ub = 454 * u;
              
              v = chromaRedValue - 128;
              vg = 183 * v;
              vr = 359 * v;
              
              for (int pixel = 0; pixel < 2; pixel++)
              {
                int deltaIndex = 2 * horizontalStep + pixel;
                lumaElementValue1 = macroBlock.DataBlocks[quadrant][lumaElementIndex1 + deltaIndex] << 8;
                lumaElementValue2 = macroBlock.DataBlocks[quadrant][lumaElementIndex2 + deltaIndex] << 8;
                
                r = Saturate5(lumaElementValue1 + vr);
                g = Saturate6(lumaElementValue1 - ug - vg);
                b = Saturate5(lumaElementValue1 + ub);
                
                PixelData[dataIndex1 + pixelDataQuadrantOffsets[quadrant] + deltaIndex] = MakeRgb(r, g, b);
                
                //?????????????????????????????????//
                //RGB24OutputPixelData[dataIndex1 + pixelDataQuadrantOffsets[quadrant] + deltaIndex].red = (unsigned char)r;
                //RGB24OutputPixelData[dataIndex1 + pixelDataQuadrantOffsets[quadrant] + deltaIndex].green = (unsigned char)g;
                //RGB24OutputPixelData[dataIndex1 + pixelDataQuadrantOffsets[quadrant] + deltaIndex].blue = (unsigned char)b;
                /////////////////////////////////////
                
                r = Saturate5(lumaElementValue2 + vr);
                g = Saturate6(lumaElementValue2 - ug - vg);
                b = Saturate5(lumaElementValue2 + ub);
                
                PixelData[dataIndex2 + pixelDataQuadrantOffsets[quadrant] + deltaIndex] = MakeRgb(r, g, b);
                
                //?????????????????????????????????//
                //RGB24OutputPixelData[dataIndex2 + pixelDataQuadrantOffsets[quadrant] + deltaIndex].red = (unsigned char)r;
                //RGB24OutputPixelData[dataIndex2 + pixelDataQuadrantOffsets[quadrant] + deltaIndex].green = (unsigned char)g;
                //RGB24OutputPixelData[dataIndex2 + pixelDataQuadrantOffsets[quadrant] + deltaIndex].blue = (unsigned char)b;
                /////////////////////////////////////
              }
            }
          }
        }
        
        imageDataOffset += 16;
      }
    }
    /////////////////////////////////////////////////
    
    void AlignStreamData()
    {
      int alignedLength;
      int actualLength;
      
      actualLength = StreamFieldBitIndex;
      
      if (actualLength > 0)
      {
        alignedLength = (actualLength & ~7);
        if (alignedLength != actualLength)
        {
          alignedLength += 0x08;
          StreamField <<= (alignedLength - actualLength);
          StreamFieldBitIndex = alignedLength;
        }
      }
    }
    
    ////////////////////////////////////////////////////
    int makeIntFromBytes(unsigned char* buffer, int index)
    {
      unsigned char b[4];
      b[0] = buffer[index];
      b[1] = buffer[index + 1];
      b[2] = buffer[index + 2];
      b[3] = buffer[index + 3];
      
      int Int32;
      
      /*
      Int32 = (Int32 << 8) + b[3];
      Int32 = (Int32 << 8) + b[2];
      Int32 = (Int32 << 8) + b[1];
      Int32 = (Int32 << 8) + b[0];*/
      
      
      ::memcpy(&Int32, b, sizeof(int));
      return Int32;
    }
    
    ////////////////////////////////////////////////////
    unsigned int PeekStreamData(unsigned char* stream, int count)
    {
      unsigned int data = 0;
      unsigned int streamField = StreamField;
      int streamFieldBitIndex = StreamFieldBitIndex;
      
      while (count > (32 - streamFieldBitIndex) && StreamIndex < (ImageStreamLength >> 2))
      {
        data = (data << (int)(32 - streamFieldBitIndex)) | (streamField >> streamFieldBitIndex);
        
        count -= 32 - streamFieldBitIndex;
        
        streamField = makeIntFromBytes(stream, StreamIndex * 4); //BitConverter.ToUInt32(stream, StreamIndex * 4);
        streamFieldBitIndex = 0;
      }
      
      if (count > 0)
      {
        data = (data << count) | (streamField >> (32 - count));
      }
      
      return data;
    }
    
    unsigned int ReadStreamData(int count)
    {
      unsigned int data = 0;
      
      
      while (count > (32 - StreamFieldBitIndex))
      {
        data = (data << (int)(32 - StreamFieldBitIndex)) | (StreamField >> StreamFieldBitIndex);
        
        count -= 32 - StreamFieldBitIndex;
        
        StreamField = makeIntFromBytes(ImageStream, StreamIndex * 4); //BitConverter.ToUInt32(ImageStream, StreamIndex * 4);
        StreamFieldBitIndex = 0;
        StreamIndex++;
      }
      
      if (count > 0)
      {
        data = (data << count) | (StreamField >> (32 - count));
        
        StreamField <<= count;
        StreamFieldBitIndex += count;
      }
      
      return data;
    }
    ////////////////////////////////////////////////////////////////////
    
    void DecodeFieldBytes(int& run, int& level, bool& last)
    {
      unsigned int streamCode = 0;
      
      int streamLength = 0; ;
      int zeroCount = 0;
      int temp = 0;
      int sign = 0;
      
      //Use the RLE and Huffman dictionaries to understand this code fragment. You can find 
      //them in the developers guide on page 34.
      //The bits in the data are actually composed of two kinds of fields:
      // - run fields - this field contains information on the number of consecutive zeros.
      // - level fields - this field contains the actual non zero value which can be negative or positive.
      //First we extract the run field info and then the level field info.
      
      streamCode = PeekStreamData(ImageStream, 32);
      
      
      //Suppose we have following bit sequence:
      //00001111.....
      // 1 - Count the number of leading zeros -> 4
      //     Coarse value lookup is thus 00001
      // 2 - Lookup the additional value, for coarse value 00001 this is 3 addtional bits
      // 3 - Calculate value of run, for coarse value 00001 this is (111) + 8
      
      zeroCount = CountLeadingZeros(streamCode); // - (1)
      streamCode <<= zeroCount + 1; // - (2) -> shift left to get rid of the coarse value
      streamLength += zeroCount + 1; // - position bit pointer to keep track off how many bits to consume later on the stream.
      
      if (zeroCount > 1)
      {
        temp = (int)(streamCode >> (32 - (zeroCount - 1))); // - (2) -> shift right to determine the addtional bits (number of additional bits is zerocount - 1)
        streamCode <<= zeroCount - 1; // - shift all of the run bits out of the way so the first bit is points to the first bit of the level field.
        streamLength += zeroCount - 1;// - position bit pointer to keep track off how many bits to consume later on the stream.
        run = temp + (1 << (zeroCount - 1)); // - (3) -> calculate run value
      }
      else
      {
        run = zeroCount;
      }
      
      
      
      //Suppose we have following bit sequence:
      //000011111.....
      // 1 - Count the number of leading zeros -> 4
      //     Coarse value lookup is thus 00001
      // 2 - Lookup the additional value, for coarse value 00001 this is 4 addtional bits (last bit is sign bit)
      // 3 - Calculate value of run, for coarse value 00001 this is (xxx) + 8, multiply by sign
      
      zeroCount = CountLeadingZeros(streamCode);
      streamCode <<= zeroCount + 1; // - (1)
      streamLength += zeroCount + 1; // - position bit pointer to keep track off how many bits to consume later on the stream.
      
      if (zeroCount == 1)
      {
        //If coarse value is 01 according to the Huffman dictionary this means EOB, so there is
        //no run and level and we indicate this by setting last to true;
        run = 0;
        last = true;
      }
      else
      {
        if(zeroCount == 0)
        {
          zeroCount = 1;
          temp = 1;
        }
        
        streamLength += zeroCount;// - position bit pointer to keep track off how many bits to consume later on the stream.
        streamCode >>= (32 - zeroCount);// - (2) -> shift right to determine the addtional bits (number of additional bits is zerocount)
        //sign = (sbyte)(streamCode & 1); // determine sign, last bit is sign 
        sign = (int)(streamCode & 1); // determine sign, last bit is sign 
        
        if (zeroCount != 0)
        {
          //temp = (sbyte)(streamCode >> 1); // take into account that last bit is sign, so shift it out of the way
          //temp += (sbyte)(1 << (zeroCount - 1)); // - (3) -> calculate run value without sign
          temp = (int)(streamCode >> 1); // take into account that last bit is sign, so shift it out of the way
          temp += (int)(1 << (zeroCount - 1)); // - (3) -> calculate run value without sign
        }
        
        level = (sign == 1) ? -temp : temp; // - (3) -> calculate run value with sign
        last = false;
      }
      
      
      ReadStreamData(streamLength);
    }
    
    /////////////////////////////////////////////////////////////////
    void ClearDataBuffer()
    {
      ::memset(dataBlockBuffer, 0, sizeof(short)*64);
    }
    ////////////////////////////////////////////////////////////////////
    void GetBlockBytes(bool acCoefficientsAvailable)
    {
      int run = 0;
      int level = 0;
      int zigZagPosition = 0;
      int matrixPosition = 0;
      bool last = false;
      
      ::memset(dataBlockBuffer, 0, 64 * sizeof(short));//Array.Clear(dataBlockBuffer, 0, dataBlockBuffer.Length);
      
      unsigned int dcCoefficient = ReadStreamData(10);
      
      if (QuantizerMode == CONST_TableQuantization)
      {
        dataBlockBuffer[0] = (short)(dcCoefficient * quantizerValues[0]);
        
        if (acCoefficientsAvailable)
        {
          DecodeFieldBytes(run, level, last);
          
          while (!last)
          {
            zigZagPosition += run + 1;
            matrixPosition = zigZagPositions[zigZagPosition];
            level *= quantizerValues[matrixPosition];
            dataBlockBuffer[matrixPosition] = (short)level;
            DecodeFieldBytes(run, level, last);
          }
        }
      }
      /*
      else
      {
        //Currently not implemented.
        //::printf("Constant quantizer mode is not yet implemented.\n");
      }*/
    }
    
    ///////////////////////////////////////////////////////////////////////////////////
    void ReadHeader()
    {
      unsigned int code = 0;
      unsigned int startCode = 0;
      
      AlignStreamData();
      
      code = ReadStreamData(22);
      
      startCode = (unsigned int)(code & ~0x1F);
      
      if (startCode == 32)
      {
        if (((code & 0x1F) == 0x1F))
        {
          PictureComplete = true;
        }
        else
        {
          if (SliceIndex++ == 0)
          {
            PictureFormat = (int)ReadStreamData(2);
            Resolution = (int)ReadStreamData(3);
            PictureType = (int)ReadStreamData(3);
            QuantizerMode = (int)ReadStreamData(5);
            FrameIndex = (int)ReadStreamData(32);
            
            switch (PictureFormat)
            {
              case kPictureFormatCIF: // (int)PictureFormats.Cif:
                Width = CONST_WidthCif << Resolution - 1;
                Height = CONST_HeightCif << Resolution - 1;
                break;
              case kPictureFormatVGA: //(int)PictureFormats.Vga:
                Width = CONST_WidthVga << Resolution - 1;
                Height = CONST_HeightVga << Resolution - 1;
                break;
            }
            
            //We assume two bytes per pixel (RGB 565)
            PixelRowSize = Width << 1;
            
            SliceCount = Height >> 4;
            BlockCount = Width >> 4;
            
            
            if(ImageSlice.MacroBlocks.size() != BlockCount)
            {
              ImageSlice.setSize(BlockCount); // = new ImageSlice(BlockCount);
              PixelData.resize(Width * Height); // = new ushort[Width * Height];
            }
            
          }
          else
          {
            QuantizerMode = (int)ReadStreamData(5);
          }
        }
      }
    }
    //////////////////////////////////////////////
    void ProcessStream()
    {
      bool blockY0HasAcComponents = false;
      bool blockY1HasAcComponents = false;
      bool blockY2HasAcComponents = false;
      bool blockY3HasAcComponents = false;
      bool blockCbHasAcComponents = false;
      bool blockCrHasAcComponents = false;
      
      //Set StreamFieldBitIndex to 32 to make sure that the first call to ReadStreamData 
      //actually consumes data from the stream
      StreamFieldBitIndex = 32;
      StreamField = 0;
      StreamIndex = 0;
      SliceIndex = 0;
      PictureComplete = false;
      
      
      while (!PictureComplete && StreamIndex < (ImageStreamLength >> 2))
      {
        ReadHeader();
        
        if (!PictureComplete)
        {
          for (int count = 0; count < BlockCount; count++)
          {
            unsigned int macroBlockEmpty = ReadStreamData(1);
            
            if (macroBlockEmpty == 0)
            {
              unsigned int acCoefficients = ReadStreamData(8);
              
              blockY0HasAcComponents = (acCoefficients >> 0 & 1) == 1;
              blockY1HasAcComponents = (acCoefficients >> 1 & 1) == 1;
              blockY2HasAcComponents = (acCoefficients >> 2 & 1) == 1;
              blockY3HasAcComponents = (acCoefficients >> 3 & 1) == 1;
              blockCbHasAcComponents = (acCoefficients >> 4 & 1) == 1;
              blockCrHasAcComponents = (acCoefficients >> 5 & 1) == 1;
              
              if ((acCoefficients >> 6 & 1) == 1)
              {
                unsigned int quantizerMode = ReadStreamData(2);
                QuantizerMode = (int)((quantizerMode < 2) ? ~quantizerMode : quantizerMode);
              }
              
              
              
              GetBlockBytes(blockY0HasAcComponents);
              InverseTransform(count, 0);
              
              
              
              GetBlockBytes(blockY1HasAcComponents);
              InverseTransform(count, 1);
              
              
              
              GetBlockBytes(blockY2HasAcComponents);
              InverseTransform(count, 2);
              
              
              
              GetBlockBytes(blockY3HasAcComponents);
              InverseTransform(count, 3);
              
              
              
              GetBlockBytes(blockCbHasAcComponents);
              InverseTransform(count, 4);
              
              
              
              GetBlockBytes(blockCrHasAcComponents);
              InverseTransform(count, 5);
              
            }
          }
          
          ComposeImageSlice();
        }
      }
    }
    ////////////////////////////////////////////////
    
     
        
    bool decodeImage(unsigned char* stream, unsigned int streamLength, ARDrone::VideoDecoder::Image& resultImage)
    {
      Width = Height = -1;
      ImageStream = stream;
      ImageStreamLength = streamLength;
      ProcessStream();
      resultImage.width = Width;
      resultImage.height = Height;
      
      if(-1 != Width && -1 != Height)
      {
        unsigned short red_mask = 0xF800;
        unsigned short green_mask = 0x7E0;
        unsigned short blue_mask = 0x1F;
        
        int length = Width * Height;
        for(int i = 0, j = 0; i < length; i++, j+=3)
        {
          unsigned short w = PixelData[i];
          unsigned char red = (PixelData[i] & red_mask) >> 11;
          unsigned char green = (PixelData[i] & green_mask) >> 5;
          unsigned char blue = (PixelData[i] & blue_mask);
          resultImage.data[j] = red << 3;
          resultImage.data[j+1] = green << 2;
          resultImage.data[j+2] = blue << 3;
        }  
        return true;
      }
      else 
      {
        ::printf("image decoding FAIL!!");
        return false;
      }
    }
  }// namespace VideoDecoder
  
} // namespace ARDrone



