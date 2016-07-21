// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.

// This sample illustrates how to use EMG data. EMG streaming is only supported for one Myo at a time.


#include "myo-osc-relay.h"

using namespace osc;

AppSettings settings;

// http://stackoverflow.com/questions/865668/how-to-parse-command-line-arguments-in-c
class InputParser{
public:
    InputParser (int &argc, char **argv){
        for (int i=1; i < argc; ++i)
            this->tokens.push_back(std::string(argv[i]));
    }
    /// @author iain
    const std::string& getCmdOption(const std::string &option) const{
        std::vector<std::string>::const_iterator itr;
        itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
        if (itr != this->tokens.end() && ++itr != this->tokens.end()){
            return *itr;
        }
        return "";
    }
    /// @author iain
    bool cmdOptionExists(const std::string &option) const{
        return std::find(this->tokens.begin(), this->tokens.end(), option)
        != this->tokens.end();
    }
private:
    std::vector <std::string> tokens;
};


void setDefaultSettings() {
    settings.interval = 20;
    settings.address = "127.0.0.1";
    settings.port = 57120;
    settings.bSendEmg = false;
    settings.bSendPose = false;
    settings.bSendQuaternion = false;
    settings.bSendAccel = false;
    settings.bSendGyro = false;
    settings.bSendLinearAccel = false;
    settings.bPrintVerbose = false;
}

void printSettings() {
    std::cout << "*** myo osc relay *** " << endl;
    std::cout << endl;
    
    std::cout << "sending OSC to " << settings.address
    << ":" << settings.port
    << " every " <<settings.interval << "ms" << endl;
    std::cout << endl;
    
    std::cout << "data to send: " << endl
    << "\temg " << settings.bSendEmg << endl
    << "\tpose " << settings.bSendPose << endl
    << "\tquaternion " << settings.bSendQuaternion << endl
    << "\taccel " << settings.bSendAccel << endl
    << "\tgyro " << settings.bSendGyro << endl
    << "\tlinaccel " << settings.bSendLinearAccel << endl;
    std::cout << endl;

    std::cout << "verbose text output " << settings.bPrintVerbose << endl;
    std::cout << endl;
}

void parseSettings(int argc, char** argv) {
    // handle command line arguments
    InputParser input(argc, argv);
    
    if(input.cmdOptionExists("-v"))
        settings.bPrintVerbose = true;
    
    const std::string &addr = input.getCmdOption("-a");
    if (!addr.empty())
        settings.address = addr;
    
    const std::string &port = input.getCmdOption("-p");
    if (!port.empty())
        settings.port = std::stoi(port);
    
    if(input.cmdOptionExists("-emg"))
        settings.bSendEmg = true;
    
    if(input.cmdOptionExists("-pose"))
        settings.bSendPose = true;
    
    if(input.cmdOptionExists("-quat"))
        settings.bSendQuaternion = true;
    
    if(input.cmdOptionExists("-accel"))
        settings.bSendAccel = true;

    if(input.cmdOptionExists("-gyro"))
        settings.bSendGyro = true;

    if(input.cmdOptionExists("-linaccel"))
        settings.bSendLinearAccel = true;

}

int main(int argc, char** argv)
{
    // default settings
    setDefaultSettings();
    
    // parse command line arguments
    parseSettings(argc, argv);

    // print to console
    printSettings();
    
    
    // We catch any exceptions that might occur below -- see the catch statement for more details.
    try {
        
        // First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
        // publishing your application. The Hub provides access to one or more Myos.
        std::string identifier = "edu.washington.dxarts-myo-osc-relay";
        std::cout << "creating hub " << identifier << std::endl;
        
        myo::Hub hub(identifier);
        
        std::cout << "tell hub not to require unlock gesture" << std::endl;
        hub.setLockingPolicy(myo::Hub::lockingPolicyNone);
        
        std::cout << "Attempting to find a Myo..." << std::endl;
        
        // Construct an instance of our DeviceListener, so that we can register it with the Hub.
        
        DeviceCollector collector;
        
        // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
        // Hub::run() to send events to all registered device listeners.
        std::cout << "Adding device listener to hub" << std::endl;
        hub.addListener(&collector);
        
        std::cout << "Starting main loop" << std::endl;
        
        // Finally we enter our main loop.
        while (1) {
            
            // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
            // In this case, we wish to update our display 50 times a second, so we run for 1000/20 milliseconds.
            hub.run(settings.interval);

            // After processing events, we print() or sendOsc() data obtained from any events that have occurred.
        
            if(settings.bPrintVerbose)
                collector.print();
            
            collector.sendOsc(settings);

        }
        
        // If a standard exception occurred, we print out its message and exit.
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
        return 1;
    }
}
