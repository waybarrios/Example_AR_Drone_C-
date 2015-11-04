// ARDrone Control in C++
// based on JavaDrone http://code.google.com/p/javadrone/
// and droneController http://dronecontroller.codeplex.com/

#include <commonc++/DatagramSocket.h++>
#include <commonc++/Mutex.h++>
#include <commonc++/Thread.h++>
#include <string>
#include <vector>
#include "../Common/MemoryLibrary.h"

namespace ARDrone
{
  //class ccxx::DatagramSocket;
  //class ccxx::Mutex;
  //class ccxx::Thread;
  
  struct NavigationData
  {
    enum eControlState
    {
      eDEFAULT = 0, 
      eINIT = 1, 
      eLANDED = 2,
      eFLYING = 3, 
      eHOVERING= 4, 
      eTEST = 5, 
      eTAKEOFF = 6, 
      eGOTOFIX = 7, 
      eLANDING = 8
    };
    
    enum eControlAlgorithm
    {
      eEulerAnglesControl=0, eAugularSpeedControl=1
    };
    
    
    struct Orientation
    {
      float pitch;
      float roll;
      float yaw;
    };
    
    struct Speed
    {
      float vx;
      float vy;
      float vz;
    };
    
    struct VisionTag
    {
      int type;
      int x;
      int y;
      int width;
      int height;
      int distance;
    };
    
    unsigned int                                 batteryLevel;
    float                                        altitude;
    ARDrone::NavigationData::Orientation         orientation;
    ARDrone::NavigationData::Speed               speed;
    ARDrone::NavigationData::eControlState       controlState;
    ARDrone::NavigationData::eControlAlgorithm   controlAlgorithm;
    std::vector<ARDrone::NavigationData::VisionTag> visionTagVector;
    // state flags
    bool          flying;
    bool          videoEnabled;
    bool          visionEnabled;
    bool          altitudeControlActive;
    bool          userFeedbackOn;           
    bool          controlReceived;
    bool          trimReceived;
    bool          trimRunning;
    bool          trimSucceeded;
    bool          navDataDemoOnly;
    bool          navDataBootstrap;
    bool          motorsDown;
    bool          gyrometersDown;
    bool          batteryTooLow;
    bool          batteryTooHigh;
    bool          timerElapsed;
    bool          notEnoughPower;
    bool          angelsOutOufRange;
    bool          tooMuchWind;
    bool          ultrasonicSensorDeaf;
    bool          cutoutSystemDetected;
    bool          PICVersionNumberOK;
    bool          ATCodedThreadOn;
    bool          navDataThreadOn;
    bool          videoThreadOn;
    bool          acquisitionThreadOn;
    bool          controlWatchdogDelayed;
    bool          ADCWatchdogDelayed;
    bool          communicationProblemOccurred;
    bool          emergency;
    
    int           sequence;
    
    const char* controlStateToString(eControlState cs)
    {
      switch(cs)
      {
        case  eDEFAULT: return "DEFAULT";
        case  eINIT: return "INIT";
        case  eLANDED: return "LANDED";
        case  eFLYING: return "FLYING";
        case  eHOVERING: return "HOVERING";
        case  eTEST: return "TEST";
        case  eTAKEOFF: return "TAKEOFF";
        case  eGOTOFIX: return "GOTOFIX";
        case  eLANDING: return "LANDING";
      }
      
      return "DONT KNOW";
    }
    
    const char* controlStateAsString()
    {
      return controlStateToString(controlState);
    }
    
    const char* visionTagAsString()
    {
      return NULL;
    }
  };

  struct ATCommand 
  {
    std::string strCommandHeader;
    std::string strCommandData;
  };
  
  class CommunicationChannel
  {
    ccxx::DatagramSocket myDatagram;
    ccxx::Mutex myMutex;
    
  public:
    CommunicationChannel();
    ~CommunicationChannel();
    
    void connectWithDroneAtAddress(const char* szDroneIpAddress, int iPort);
    void disconnectFromDrone();
    bool isConnectedWithDrone();
    
    void setTimeout(int t);
    
    void send(unsigned char* bytes, unsigned int length);
    void receive(unsigned char* bytes, unsigned int& bufferLength);

    void sendAT(const char* szHeader, const char* szDetail, unsigned int mssleep=100);
    
    unsigned int nextATSequence();
    ARDrone::ATCommand lastATCommand();
  };
  
  class Controller
  {
    ARDrone::CommunicationChannel myCommunicationChannel;
  public:
    Controller();
    ~Controller();
    
    void connectWithDroneAtAddress(const char* szDroneIpAddress);
    
    void takeOff();
    void land();
    void hover();
    
    void requestNavigationData();
    void requestVideoData();
    void disableAdaptiveVideo();
    
    void switchToFrontCamera();
    void switchToDownCamera();
    
    void sendControlParameters(int enable, float pitch, float roll, float yaw, float gaz);
    void sendWatchDogReset();
    void sendFlatTrim();
    void sendEmergencyShutdown();
    void sendLastCommand();
  };
  
  class NavigationDataReceiver :public ccxx::Thread
  {
    ARDrone::CommunicationChannel myCommunicationChannel;
    NavigationData myNavData;
    ccxx::Mutex myMutex;
    std::string myDroneAddress;
    ARDrone::Controller* myController;
  public:
    NavigationDataReceiver(ARDrone::Controller* pController, const char* szDroneIpAddress);
    ~NavigationDataReceiver() throw ();
    
    void copyDataTo(ARDrone::NavigationData& data);
  protected:
    void run();
    bool parse(MemoryLibrary::Buffer& buffer);
    bool parseState(int state);
    bool parseNavigation(MemoryLibrary::Buffer& buffer, int offset);
    bool parseVision(MemoryLibrary::Buffer& buffer, int offset);
  };
  
  
  namespace VideoDecoder
  {
    struct Image
    {
      unsigned char data[921600]; //640x480x3 MAX out
      int width;
      int height;
    };
    
    bool decodeImage(unsigned char* stream, unsigned int streamLength, ARDrone::VideoDecoder::Image& resultImage);
  }  //namespace VideoDecoder
  
  class VideoDataReceiver :public ccxx::Thread
  {
    ARDrone::CommunicationChannel myCommunicationChannel;
    ccxx::Mutex myMutex;
    std::string myDroneAddress;
    ARDrone::Controller* myController;
    unsigned char myVideoData[921600]; //640x480x3 MAX out
    unsigned int videoDataLength;
  public:
    VideoDataReceiver(ARDrone::Controller* pController, const char* szDroneIpAddress);
    ~VideoDataReceiver() throw ();
    
    void copyDataTo(ARDrone::VideoDecoder::Image& resultImage); 
  protected:
    void run();
  }; 
  
  class Drone
  {
    ARDrone::Controller* myController;
    ARDrone::VideoDataReceiver* myVideoDataReceiver;
    ARDrone::NavigationDataReceiver* myNavigationDataReceiver;
    
  public:
    Drone();
    ~Drone();
    
    bool start(const char* szDroneAddress="192.168.1.1");
    void stop();
    
    ARDrone::Controller& controller();
    ARDrone::VideoDataReceiver& videoDataReceiver();
    ARDrone::NavigationDataReceiver& navigationDataReceiver();
  };
  
  
}//namespace ARDrone
