//
//  DeviceCollector.hpp
//  myo-osc-relay
//
//  Created by Robert Twomey on 7/13/16.
//  Copyright © 2016 Thalmic Labs. All rights reserved.
//

#ifndef DataCollector_hpp
#define DataCollector_hpp

#include <stdio.h>
#include <array>

#include <myo/myo.hpp>

#include "myo-osc-relay.h"
#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"

#define OUTPUT_BUFFER_SIZE 1024

class DeviceCollector;

class AppSettings {
public:
    
    // myo sampling rate
    int interval;
    
    // osc
    std::string address;
    int port;
    
    bool bSendPose;
    bool bSendEmg;
    bool bSendQuaternion;
    bool bSendAccel;
    bool bSendGyro;
    bool bSendLinearAccel;
    
    bool bPrintVerbose;
};

using namespace myo;
using namespace std;

typedef Vector3<float> Vector3f;
typedef Quaternion<float> Quaternion4f;

class Device{
    friend DeviceCollector;
    
public:
    
    Device();
    
    void reset();
    
    int getId() ;
    void setId( int v);
    
    Vector3f getAccel();
    void setAccel(Vector3f v) ;
    
    Vector3f getGyro();
    void setGyro(Vector3f v) ;
    
    Quaternion4f getQuaternion() ;
    void setQuaternion(Quaternion4f v) ;
    
    myo::Pose getPose() ;
    void setPose(myo::Pose v) ;
    
    bool getOnArm() ;
    void setOnArm(bool b) ;
    bool getIsUnlocked() ;
    void setIsUnlocked(bool b) ;
    bool getIsConnect();
    void setIsConnect(bool b);
    
    myo::Arm getWhichArm();
    std::string getWhichArmAsString() const;
    void setWhichArm(myo::Arm v) ;
    
    std::vector<uint> getEmgSamples() ;
    void setEmgSamples(std::vector<uint> vals);
    
    float getRoll();
    void setRoll(float v);
    
    float getPitch();
    void setPitch(float v);
    
    float getYaw();
    void setYaw(float v) ;
    
    Vector3f getLinearAccel();
    void setLinearAccel(Vector3f v) ;
    
    float getGravity();
    void setGravity(float val);
    
    void print();
    void sendOsc(const char* address, int port, bool bSendQuaternion, bool bSendAccel, bool bSendGyro, bool bSendLinearAccel, bool bSendEmg);
    
    
    
    //    float getLastTimef() { return last_timef; }
    //    void setLastTimef(float t) { last_timef = t; }
    //    float getTimef() { return timef; }
    //    void setTimef(float t) { timef = t; }
    
protected:
    myo::Myo * myo;
    
    int id;
    bool isConnect;
    bool isUnlocked;
    bool onArm;
    myo::Arm whichArm;
    
    myo::Pose currentPose;
    
    int roll_w, pitch_w, yaw_w;
    float roll, pitch, yaw;
    
    Vector3f accel;
    Vector3f gyro;
    Vector3f linear_accel;
    
    Quaternion4f q;
    
    std::vector<uint> emgSamples;
    
    float gravity = 0.98;
    
    string poseString;
    
    //    ofMatrix4x4 currentRotationMatrix;
    //    float last_timef;
    //    float timef;
};

class DeviceCollector : public myo::DeviceListener {
    
    friend Device;
public:
    DeviceCollector();
    //    void onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion);
    //    void onUnpair(myo::Myo* myo, uint64_t timestamp);
    //    void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg);
    //    void onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& accel);
    //    void onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& gyro);
    //    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat);
    //    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose);
    //    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection);
    //    void onArmUnsync(myo::Myo* myo, uint64_t timestamp);
    //    void onConnect(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion);
    //    void onDisconnect(myo::Myo* myo, uint64_t timestamp) ;
    //    void onUnlock(myo::Myo* myo, uint64_t timestamp);
    //    void onLock(myo::Myo* myo, uint64_t timestamp);
    
    void onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion);
    void onUnpair(myo::Myo* myo, uint64_t timestamp);
    void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg);
    void onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& accel);
    void onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& gyro);
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat);
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose);
    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection);
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp);
    void onConnect(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion);
    void onDisconnect(myo::Myo* myo, uint64_t timestamp) ;
    void onUnlock(myo::Myo* myo, uint64_t timestamp);
    void onLock(myo::Myo* myo, uint64_t timestamp);
    
    Device * findDevice(myo::Myo* myo);
    
    void print();
//    void sendOsc(const char* address, int port, AppSettings oscsettings);
    void sendOsc(AppSettings settings);
    
    std::vector<Device*> devices;
};

#endif /* DeviceCollector_hpp */
