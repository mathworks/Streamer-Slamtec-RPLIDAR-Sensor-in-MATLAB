// Slamtec RPLIDAR SDK MATLAB/mex interface

// Copyright 2023 The MathWorks, Inc.

#include "mex.hpp"
#include "mexAdapter.hpp"
#include "MatlabDataArray.hpp"

#include <string.h>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace sl;

class MexFunction : public matlab::mex::Function {
  public:
    MexFunction() {}
    ~MexFunction();
    
    // To open/close device connection
    matlab::data::StructArray open(const std::string &port, const uint32_t &baudrate);
    void close();

    // To start/stop sensor scanning
    void startScanning();
    void stopScanning();

    // read scan data
    void scan(matlab::mex::ArgumentList &outputs);

    // reset device
    void reset() {
        if(isConnected()) {
            throwErrorIf(SL_IS_OK(lidarDriver->reset()), "Error resetting device.");
        }
    }

    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs);

  private:
    
    void processArgs(matlab::mex::ArgumentList &inputs, matlab::mex::ArgumentList &outputs);

    bool isConnected();

    bool changeScanMode(double scanModeInput);

    bool changeMotorSpeed(double motorSpeedInput);

    void setScanMode(const uint16_t typicalScanMode) {
        selectedScanMode = typicalScanMode;
    }

    void setMotorSpeed(const uint16_t motorSpeed) { 
        selectedMotorSpeed = motorSpeed;
    }

    void throwError(const std::string &errorStr = "unknown error");
    void throwErrorIf(bool condition, const std::string &errorStr = "unknown error", const bool closeConnection = false);

    void validateScanMode(const uint16_t &scanMode) {
        sprintf(buffer, "Error, scan mode cannot be > %d", maxScanMode);
        throwErrorIf(scanMode <= maxScanMode, buffer);
    }

    void validateMotorSpeed(const uint16_t &motorSpeed) {
        sprintf(buffer, "Error, motor speed must be in the range [ %d, %d ]", minMaxMotorSpeeds[0], minMaxMotorSpeeds[1]);
        throwErrorIf(((motorSpeed >= minMaxMotorSpeeds[0]) && (motorSpeed <= minMaxMotorSpeeds[1])), buffer);
    }
    
    bool checkLidarHealth(ILidarDriver * lidarDriver);
    matlab::data::StructArray getDeviceDetails(ILidarDriver * lidarDriver);
    std::string getStringInput(const matlab::data::TypedArray<matlab::data::MATLABString> &input);

    std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
    matlab::data::ArrayFactory factory;

    bool isScanning = false;

    IChannel* serialPortChannel = nullptr;
    ILidarDriver* lidarDriver = nullptr;

    LidarScanMode currentScanMode;

    uint16_t maxScanMode = 0;
    uint16_t minMaxMotorSpeeds[2]{};

    uint16_t selectedScanMode = 0;
    uint16_t selectedMotorSpeed = 0;

    char buffer[200]; // for sprintf(..);
};

bool MexFunction::changeScanMode(double scanModeInput) {

    throwErrorIf(isConnected(), "Sensor not connected.");

    setScanMode(static_cast<uint16_t>(scanModeInput));

    return true;
}

bool MexFunction::changeMotorSpeed(double motorSpeedInput) {
    if(!isConnected()) {
        throwError("Sensor not connected.");
    }
    setMotorSpeed(static_cast<uint16_t>(motorSpeedInput));
    sl_result op_result = lidarDriver->setMotorSpeed(static_cast<uint16_t>(motorSpeedInput));

    return SL_IS_OK(op_result);
}

MexFunction::~MexFunction() {
    close(); 
}

bool MexFunction::isConnected() {
    // Lidar driver is not null and is connected
    return (lidarDriver && (lidarDriver->isConnected()));
}

void MexFunction::throwErrorIf(bool condition, const std::string &errorStr, const bool closeConnection) {
    if(!condition) { // if condition fails, throw error
        if(closeConnection) {
            close();
        }
        throwError(errorStr);
    }
}

void MexFunction::throwError(const std::string &errorStr) {
    matlabPtr->feval(u"error", 0, 
                std::vector<matlab::data::Array>({factory.createScalar(errorStr)}));
}

void MexFunction::scan(matlab::mex::ArgumentList &outputs) {
    if(!isConnected()) {
        outputs[0] = std::move(factory.createArray<double>({ 1, 0 }));
        outputs[1] = std::move(factory.createArray<double>({ 1, 0 }));
        return;
    }

    // Error if not scanning
    throwErrorIf(isScanning, "Error starting scan.");

    sl_result     op_result;

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    
    size_t   count = _countof(nodes);

    op_result = lidarDriver->grabScanDataHq(nodes, count);
    throwErrorIf(SL_IS_OK(op_result), "Error getting scan data.");

    if (SL_IS_OK(op_result)) {
        lidarDriver->ascendScanData(nodes, count);
        float frequency = 0;
        lidarDriver->getFrequency(currentScanMode, nodes, count, frequency);
        
        matlab::data::TypedArray<double> scanReadings = factory.createArray<double>({ 3, count });

        for (int pos = 0; pos < (int)count ; ++pos) {
            
            double theta = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
            double theta_rad = theta*(22.0/7.0)/180.0;

            scanReadings[0][pos] = nodes[pos].dist_mm_q2/4.0e3f;  // in meters
            scanReadings[1][pos] = theta_rad; // in radians
            scanReadings[2][pos] = nodes[pos].quality;
        }

        outputs[0] = std::move(scanReadings);
        outputs[1] = std::move(factory.createScalar<double>(frequency));
    }
    else {
        outputs[0] = std::move(factory.createArray<double>({ 1, 0 }));
        outputs[1] = std::move(factory.createArray<double>({ 1, 0 }));
    }

}


matlab::data::StructArray MexFunction::open(const std::string &port, const uint32_t &baudrate) {

    // Create lidar driver
    lidarDriver = *createLidarDriver();
    
    throwErrorIf((bool)lidarDriver, "Cannot create Lidar driver object.");
    
    sl_result     op_result;

    serialPortChannel = (*createSerialPortChannel(port.c_str(), baudrate));
    throwErrorIf((bool)serialPortChannel, "Cannot create Serial port channel.");


    op_result = lidarDriver->connect(serialPortChannel);
    throwErrorIf(SL_IS_OK(op_result), "Error, cannot bind to the specified serial port.", true);

    sl_lidar_response_device_info_t devinfo;
    op_result = lidarDriver->getDeviceInfo(devinfo);
    throwErrorIf(SL_IS_OK(op_result), "Error, cannot get device info. Select the correct Port-Baudrate values for the sensor.", true);

    return getDeviceDetails(lidarDriver);
}

matlab::data::StructArray MexFunction::getDeviceDetails(ILidarDriver * lidarDriver)
{
    matlab::data::ArrayFactory f;
    matlab::data::StructArray S = f.createStructArray({ 1,1 },
                                        { "Model", "FirmwareVersion", "HardwareVersion", "SerialNumber", "RPM",
                                          "TypicalScanMode", "SupportedScanModes" });
    if(!isConnected()) {
        return S;
    }
    sl_result     op_result;
    // Device Health
    sl_lidar_response_device_health_t healthinfo;

    op_result = lidarDriver->getHealth(healthinfo);
    throwErrorIf(SL_IS_OK(op_result), "Error getting device health.");

    char buffer[200];
    
    if(healthinfo.status == SL_LIDAR_STATUS_ERROR) {
        // try software reboot once and get health again
        reset();
        delay(200);
        op_result = lidarDriver->getHealth(healthinfo);
        throwErrorIf(SL_IS_OK(op_result), "Error getting device health.");
    }
    
    throwErrorIf(healthinfo.status != SL_LIDAR_STATUS_ERROR, "Device health status ERROR. Try rebooting the device.");

    // Device Info
    sl_lidar_response_device_info_t devinfo;

    op_result = lidarDriver->getDeviceInfo(devinfo);
    throwErrorIf(SL_IS_OK(op_result), "Error getting device info.");

    for (int pos = 0; pos < 16 ;++pos) {
        sprintf(buffer+pos*2, "%02X", devinfo.serialnum[pos]);
    }

    S[0]["Model"] = f.createScalar<double>(devinfo.model);
    S[0]["SerialNumber"] = f.createCharArray(buffer);
    sprintf(buffer, "%d.%02d",  devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
    S[0]["FirmwareVersion"] = f.createCharArray(buffer);
    S[0]["HardwareVersion"] = f.createScalar<double>(devinfo.hardware_version);
    
    // SCAN modes
    std::vector<LidarScanMode> scanModes;
    op_result = lidarDriver->getAllSupportedScanModes(scanModes);
    throwErrorIf(SL_IS_OK(op_result), "Error getting supported scan modes.");

    maxScanMode = scanModes[scanModes.size()-1].id;

    matlab::data::TypedArray<matlab::data::MATLABString> supportedScanModes = f.createArray<matlab::data::MATLABString>({ 1, scanModes.size() });
    size_t ind = 0;
    for(const auto& ele : scanModes) {
        supportedScanModes[ind++] = std::string(ele.scan_mode);
    }

    // Typical scan mode
    sl_u16 typicalScanMode;
    op_result = lidarDriver->getTypicalScanMode(typicalScanMode);
    throwErrorIf(SL_IS_OK(op_result), "Error getting typical scan modes.");

    currentScanMode = scanModes[typicalScanMode];

    setScanMode(typicalScanMode);

    S[0]["SupportedScanModes"] = supportedScanModes;
    S[0]["TypicalScanMode"] = f.createCharArray(scanModes[typicalScanMode].scan_mode);

    // Device Motor info
    LidarMotorInfo motor_info;
    motor_info.motorCtrlSupport = sl::MotorCtrlSupport::MotorCtrlSupportRpm;

    op_result = lidarDriver->getMotorInfo (motor_info, 1000);

    throwErrorIf(SL_IS_OK(op_result), "Error getting motor info.");
    minMaxMotorSpeeds[0] = motor_info.min_speed;
    minMaxMotorSpeeds[1] = motor_info.max_speed;

    S[0]["RPM"] = f.createArray<double>({ 1, 3 }, { double(motor_info.desired_speed), double(motor_info.min_speed), double(motor_info.max_speed) });

    setMotorSpeed(motor_info.desired_speed);

    return S;
}


void MexFunction::stopScanning() {
    // Stop motor/scanning
    if(isConnected()) {
        throwErrorIf(SL_IS_OK(lidarDriver->stop()), "Error stopping scan.");
        delay(200);
        isScanning = false;
    }
}


void MexFunction::startScanning() {
    if(isConnected() && !isScanning) {
        sl_result     op_result;

        // Check sensor health before scanning
        throwErrorIf(checkLidarHealth(lidarDriver), "Error, device health returned error. Try rebooting the device.");

        op_result = lidarDriver->startScanExpress(0, selectedScanMode, 0, &currentScanMode);
        throwErrorIf(SL_IS_OK(op_result), "Error starting scan.");

        op_result = lidarDriver->setMotorSpeed(selectedMotorSpeed);
        throwErrorIf(SL_IS_OK(op_result), "Error setting motor speed after start scan.");
        isScanning = true;
    }
}

void MexFunction::close() {
    
    if(isConnected()) {
        stopScanning();
        
        lidarDriver->disconnect();

        delete lidarDriver;
        lidarDriver = nullptr;

        if(serialPortChannel) {
            serialPortChannel->close();
            serialPortChannel = NULL;
        }
        
    }
    else if(lidarDriver) {
        // lidarDriver created but not connected to port/device
        
        lidarDriver->disconnect();
        delete lidarDriver;
        lidarDriver = nullptr;

        if(serialPortChannel) {
            serialPortChannel->close();
            serialPortChannel = NULL;
        }
    }

}

bool MexFunction::checkLidarHealth(ILidarDriver * lidarDriver)
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = lidarDriver->getHealth(healthinfo);
    char buffer[200];
    if (SL_IS_OK(op_result)) {
        
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            return false;
        } else {
            return true;
        }

    } else {
        return false;
    }
}


std::string MexFunction::getStringInput(const matlab::data::TypedArray<matlab::data::MATLABString> &input) {
    
    return std::string(input[0]);
}

void MexFunction::processArgs( matlab::mex::ArgumentList &inputs, matlab::mex::ArgumentList &outputs) {

    size_t numInputs = inputs.size();

    throwErrorIf(numInputs > 0, "Not enough inputs.");

    std::string cmd_str = "";
    bool error = false;
    if(inputs[0].getType() != matlab::data::ArrayType::MATLAB_STRING) {
        throwError("Wrong inputs..\n");
    }

    cmd_str = getStringInput(inputs[0]);

    std::string port = "";
    double baudrate = 0;

    double scanModeInput = 0;
    double motorSpeedInput = 0;

    if(cmd_str.compare("open") == 0) {
        if(numInputs == 3 ) {
            // out_struct = mex("start", "port", baudrate)
            if(inputs[1].getType() != matlab::data::ArrayType::MATLAB_STRING) {
                error = true;
            }
            if(inputs[2].getType() != matlab::data::ArrayType::DOUBLE) {
                error = true;
            }

            port = getStringInput(inputs[1]);
            baudrate = matlab::data::TypedArray<double>(inputs[2])[0];

            matlab::data::StructArray s = open(port, baudrate);
            outputs[0] = std::move(s);
        }
        else {
            error = true;
        }
    }
    else if(cmd_str.compare("scan") == 0) {
        if(numInputs == 1 ) {
            // mex("scan")
            scan(outputs);
        }
        else {
            error = true;
        }
    }
    else if(cmd_str.compare("close") == 0) {
        if(numInputs == 1 ) {
            // mex("stop")
            close();
        }
        else {
            error = true;
        }
    }
    else if(cmd_str.compare("reset") == 0) {
        if(numInputs == 1 ) {
            // mex("reset")
            reset();
        }
        else {
            error = true;
        }
    }
    else if(cmd_str.compare("startscanning") == 0) {
        if(numInputs == 1 ) {
            // mex("startscanning")
            startScanning();
        }
        else {
            error = true;
        }
    }
    else if(cmd_str.compare("stopscanning") == 0) {
        if(numInputs == 1 ) {
            // mex("stopscanning")
            stopScanning();
        }
        else {
            error = true;
        }
    }
    else if(cmd_str.compare("setscanmode") == 0) {
        if(numInputs == 2 ) {
            // mex("setScanMode", scanId)
            if(inputs[1].getType() != matlab::data::ArrayType::DOUBLE) {
                error = true;
            }
            scanModeInput = matlab::data::TypedArray<double>(inputs[1])[0];
            validateScanMode(scanModeInput);
            
            bool status = changeScanMode(scanModeInput); 
            throwErrorIf(status, "Error setting scan mode.");
        }
        else {
            error = true;
        }
    }
    else if(cmd_str.compare("setmotorspeed") == 0) {
        if(numInputs == 2 ) {
            // mex("setMotorSpeed", rpm)
            if(inputs[1].getType() != matlab::data::ArrayType::DOUBLE) {
                error = true;
            }
            motorSpeedInput = matlab::data::TypedArray<double>(inputs[1])[0];
            validateMotorSpeed(motorSpeedInput);

            bool status = changeMotorSpeed(motorSpeedInput); 
            throwErrorIf(status, "Error setting motor speed.");
        }
        else {
            error = true;
        }
    }
    else {
        error = true;
    }

    if(error) {
        throwError("Wrong inputs..\n");
    }
}

void MexFunction::operator()(matlab::mex::ArgumentList outputs,
                             matlab::mex::ArgumentList inputs) {

    processArgs(inputs, outputs);
}
