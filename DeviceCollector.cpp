//
//  DeviceCollector
//  myo-osc-relay
//
//  Created by Robert Twomey on 7/13/16.
//  Copyright © 2016 Thalmic Labs. All rights reserved.
//

#include "DeviceCollector.hpp"

#include <array>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace osc;

#pragma mark - Device

Device::Device() : onArm(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose() {
    
    q = Quaternion4f(0, 0, 0, 0);
    
    isUnlocked = false;
    
    emgSamples.resize(8);//(0);
    
}

void Device::reset(){
    roll_w = 0;
    pitch_w = 0;
    yaw_w = 0;
    onArm = false;
    //    whichArm =
    isUnlocked = false;
    emgSamples.resize(8);
}

int Device::getId() { return id; }
void Device::setId( int v) { id = v; }

Vector3f Device::getAccel(){ return accel; }
void Device::setAccel(Vector3f v) { accel = v; }

Vector3f Device::getGyro(){ return gyro; }
void Device::setGyro(Vector3f v) { gyro = v; }

Quaternion4f Device::getQuaternion() { return q; }
void Device::setQuaternion(Quaternion4f v) { q = v; }

myo::Pose Device::getPose() { return currentPose; }
void Device::setPose(myo::Pose v) { currentPose = v; }

bool Device::getOnArm() { return onArm; }
void Device::setOnArm(bool b) { onArm = b; }
bool Device::getIsUnlocked() { return isUnlocked; }
void Device::setIsUnlocked(bool b) { isUnlocked = b; }
bool Device::getIsConnect() { return isConnect; }
void Device::setIsConnect(bool b) { isConnect = b; }

myo::Arm Device::getWhichArm(){ return whichArm; }

std::string Device::getWhichArmAsString() const {
    switch (whichArm) {
        case myo::Arm::armLeft:
            return "left";
            break;
        case myo::Arm::armRight:
            return "right";
            break;
        default:
            return "unknown";
    }
}

void Device::setWhichArm(myo::Arm v) { whichArm = v; }

std::vector<int> Device::getEmgSamples() { return emgSamples; }
void Device::setEmgSamples(std::vector<int> vals) {
    if ( emgSamples.size() == vals.size() ) {
        for ( int i=0; i<emgSamples.size(); i++ ) {
            emgSamples[i] = vals[i];
        }
    }
}

float Device::getRoll() { return roll; }
void Device::setRoll(float v) { roll = v; }

float Device::getPitch() { return pitch; }
void Device::setPitch(float v) { pitch = v; }

float Device::getYaw() { return yaw; }
void Device::setYaw(float v) { yaw = v; }

Vector3f Device::getLinearAccel(){ return linear_accel; }
void Device::setLinearAccel(Vector3f v) { linear_accel = v; }

float Device::getGravity() { return gravity; }
void Device::setGravity(float val) { gravity = val; }

// We define this function to print the current values that were updated by the on...() functions above.
void Device::print()
{
    
    // Clear the current line
    std::cout << '\r';
    
    // Print out the orientation. Orientation data is always available, even if no arm is currently recognized.
    std::cout << '[' << std::string(roll_w, '*') << std::string(18 - roll_w, ' ') << ']'
    << '[' << std::string(pitch_w, '*') << std::string(18 - pitch_w, ' ') << ']'
    << '[' << std::string(yaw_w, '*') << std::string(18 - yaw_w, ' ') << ']';
    
//    std::cout << onArm;
    
//    if (onArm) {
        // Print out the lock state, the currently recognized pose, and which arm Myo is being worn on.
        
        // Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
        // output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
        // that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
        std::string poseString = currentPose.toString();
        
        std::cout << '[' << (isUnlocked ? "unlocked" : "locked  ") << ']'
        << '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
        << '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
        
        
        // Print out the EMG data.
        for (size_t i = 0; i < emgSamples.size(); i++) {
            std::ostringstream oss;
            oss << static_cast<int>(emgSamples[i]);
            std::string emgString = oss.str();
            
            std::cout << '[' << emgString << std::string(4 - emgString.size(), ' ') << ']';
        }
        
//    } else {
//        // Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
//        std::cout << '[' << std::string(8, ' ') << ']' << "[?]" << '[' << std::string(14, ' ') << ']';
//    }
    
    std::cout << std::flush;
}

void Device::sendOsc(const char* address, int port, bool bSendQuaternion, bool bSendAccel, bool bSendGyro, bool bSendLinearAccel, bool bSendEmg) {
    // message format:
    // /myo, id, isconnected, onarm, whicharm, isunlocked, pose,
    //   quaternion-x, qy, qz, qw
    
    //    ofxOscMessage m;
    //
    //    m.setAddress("/myo");
    //
    //    m.addIntArg(thisDevice->getId());
    //    m.addIntArg(thisDevice->getIsConnect());
    //    //    m.addIntArg(thisDevice->getIsUnlocked());
    //    m.addIntArg(thisDevice->getOnArm());
    //    m.addStringArg(thisDevice->getWhichArmAsString());
    //    m.addStringArg(thisDevice->getPose().toString());
    //
    //    m.addFloatArg(thisDevice->getQuaternion().x());
    //    m.addFloatArg(thisDevice->getQuaternion().y());
    //    m.addFloatArg(thisDevice->getQuaternion().z());
    //    m.addFloatArg(thisDevice->getQuaternion().w());
    //
    //    //    m.addFloatArg(thisDevice->getAccel().x);
    //    //    m.addFloatArg(thisDevice->getAccel().y);
    //    //    m.addFloatArg(thisDevice->getAccel().z);
    //
    //    for ( int j=0; j<8; j++ ) {
    //        m.addFloatArg(thisDevice->getEmgSamples()[j]);
    //    }
    //
    //    sender.sendMessage(m, false);
    
    // start OSC
    UdpTransmitSocket transmitSocket( IpEndpointName( address, port ) );
    
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
    
    p << osc::BeginBundleImmediate
    << osc::BeginMessage( "/myo" )
    << id
    << isConnect
    << onArm
    << getWhichArmAsString().c_str()
    << currentPose.toString().c_str();
    if(bSendQuaternion)
        p << q.x() << q.y() << q.z() << q.w();
    if(bSendAccel)
        p << accel.x() << accel.y() << accel.z();
    if(bSendGyro)
        p << gyro.x() << gyro.y() << gyro.z();
    if(bSendLinearAccel)
        p << linear_accel.x() << linear_accel.y() << linear_accel.z();
    if(bSendEmg)
        for (size_t i = 0; i < emgSamples.size(); i++)
            p << static_cast<int>(emgSamples[i]);
    
    p << osc::EndMessage
    << osc::EndBundle;
//    << true << 23 << (float)3.1415 << "hello" << osc::EndMessage
//    << osc::BeginMessage( "/test2" )
//    << true << 24 << (float)10.8 << "world" << osc::EndMessage
//    << osc::EndBundle;
    
    transmitSocket.Send( p.Data(), p.Size() );
    
}

#pragma mark - DeviceCollector

DeviceCollector::DeviceCollector(){
}

void DeviceCollector::onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
{
    // enable EMG streaming
    myo->setStreamEmg(myo::Myo::streamEmgEnabled);
    
    //    // hold unlocked
    //    myo->unlock(myo::Myo::unlockHold);

//    myo->vibrate(myo::Myo::vibrationShort);
//    std::cout << "onPair found device" << std::endl;

    if ( !findDevice(myo) ) {
        Device * device = findDevice(myo);
        device = new Device();
        device->myo = myo;
        device->id = devices.size();
        devices.push_back(device);
        //        std::cout << "Paired with " << device->id << "." << std::endl;
//        ofLog(OF_LOG_NOTICE, "Paired with myo " + ofToString(device->id));
        cout << "Paired with myo " << device->id << endl;
        device->myo->vibrate(myo::Myo::vibrationShort);
        device->myo->vibrate(myo::Myo::vibrationMedium);
        
    }
}

void DeviceCollector::onUnpair(myo::Myo* myo, uint64_t timestamp)
{
    Device * device = findDevice(myo);
    if ( device ) {
        device->reset();
    }
}

void DeviceCollector::onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
{
    Device * device = findDevice(myo);
    if ( device ) {
        for (int i = 0; i < 8; i++) {
            device->emgSamples[i] = emg[i];
        }
    }
}

void DeviceCollector::onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& accel)
{
    Device * device = findDevice(myo);
    if ( device ) {
        device->accel = accel;
        //        device->accel.x = accel.x();
//        device->accel.y = accel.y();
//        device->accel.z = accel.z();
    }
}


void DeviceCollector::onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& gyro)
{
    
    Device * device = findDevice(myo);
    if ( device ) {
        device->gyro = gyro;
//        device->gyro.x = gyro.x();
//        device->gyro.y = gyro.y();
//        device->gyro.z = gyro.z();
    }
    
}

void DeviceCollector::onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
{
    Device * device = findDevice(myo);
    if ( device ) {
        using std::atan2;
        using std::asin;
        using std::sqrt;
        
        // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
        float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                           1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
        float pitch = asin(2.0f * (quat.w() * quat.y() - quat.z() * quat.x()));
        float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                          1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));
        
        device->q = quat;//set(quat.x(), quat.y(), quat.z(), quat.w());
        
        
        device->roll = roll;
        device->pitch = pitch;
        device->yaw = yaw;
        
        
        // Convert the floating point angles in radians to a scale from 0 to 20.
        device->roll_w = static_cast<int>((roll + (float)M_PI)/(M_PI * 2.0f) * 18);
        device->pitch_w = static_cast<int>((pitch + (float)M_PI/2.0f)/M_PI * 18);
        device->yaw_w = static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
        
        float gyoro_g = sqrt(device->gyro.x()*device->gyro.x() + device->gyro.y()*device->gyro.y() + device->gyro.z()*device->gyro.z());
        float g = sqrt(device->accel.x()*device->accel.x() + device->accel.y()*device->accel.y() + device->accel.z()*device->accel.z());
        //            cout << gyoro_g << endl;
        //            cout << g << endl;
        if ( gyoro_g <= 0.2 ) device->gravity = g;
        
//        Quaternion4f q;
//        q = quat;
//        //.set(quat.x(), quat.z(), quat.y(), quat.w());
//        Vector3f linear_accel;
//        ofMatrix4x4 mat;
//        mat.translate(ofVec3f(0,device->gravity,0));
//        mat.rotate(q);
//        ofVec3f trans = mat.getTranslation();
//        
//        linear_accel = device->getAccel();
//        linear_accel.x = linear_accel.x - trans.x;
//        linear_accel.y = linear_accel.y - trans.z;
//        linear_accel.z = linear_accel.z - trans.y;
//
//        device->linear_accel.set(linear_accel);
        //            cout << device->getAccel() << endl;
        //            cout << mat.getTranslation() << endl;
        //            cout << linear_accel << endl;
    }
}

void DeviceCollector::onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
{
    Device * device = findDevice(myo);
    if ( device ) {
        device->currentPose = pose;
        //            if (pose == myo::Pose::fist) {
        //                myo->vibrate(myo::Myo::vibrationShort);
        //            }
        
    }
}

void DeviceCollector::onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection)
{
    Device * device = findDevice(myo);
    if ( device ) {
        device->onArm = true;
        device->whichArm = arm;
    }
}

void DeviceCollector::onArmUnsync(myo::Myo* myo, uint64_t timestamp)
{
    
    Device * device = findDevice(myo);
    if ( device ) {
        device->onArm = false;
    }
}

void DeviceCollector::onConnect(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion) {
    myo->setStreamEmg(myo::Myo::streamEmgEnabled);
    Device * device = findDevice(myo);
    if ( device ) {
        device->isConnect = true;
    }
}

void DeviceCollector::onDisconnect(myo::Myo* myo, uint64_t timestamp) {
    Device * device = findDevice(myo);
    if ( device ) {
        device->isConnect = false;
    }
}

void DeviceCollector::onUnlock(myo::Myo* myo, uint64_t timestamp)
{
    Device * device = findDevice(myo);
    if ( device ) {
        device->isUnlocked = true;
    }
}

void DeviceCollector::onLock(myo::Myo* myo, uint64_t timestamp)
{
    Device * device = findDevice(myo);
    if ( device ) {
        device->isUnlocked = false;
    }
}

Device * DeviceCollector::findDevice(myo::Myo* myo){
    for (int i = 0; i < devices.size(); ++i) {
        if (devices[i]->myo == myo) {
            return devices[i];
        }
    }
    return 0;
}

// We define this function to print the current values that were updated by the on...() functions above.
void DeviceCollector::print()
{
    for ( int i=0; i < devices.size(); i++ ) {
        
        Device *thisDevice = devices[i];
        
        thisDevice->print();
    };
}

void DeviceCollector::sendOsc(AppSettings settings) {
    
    for ( int i=0; i < devices.size(); i++ ) {
        
        Device *thisDevice = devices[i];
        
        thisDevice->sendOsc(settings.address.c_str(), settings.port,
                            settings.bSendQuaternion,
                            settings.bSendAccel,
                            settings.bSendGyro,
                            settings.bSendLinearAccel,
                            settings.bSendEmg);
    };
    
}
