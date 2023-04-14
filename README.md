# Streamer for Slamtec RPLIDAR Sensor in MATLAB&reg;
Package for streaming Slamtec [RPLIDAR Laser scanner](https://www.slamtec.com/en/S2) data into MATLAB.

# Support platforms
- Windows&reg;

# Setup
- Download & Install Slamtec RPLIDAR SDK for C++
- Install Slamtec RPLIDAR device driver software (if not already installed)
- Compile mex file from MATLAB and run demo script

# MathWorks&reg; Products (https://www.mathworks.com)
Requires MATLAB release R2020b or newer
- [Lidar Toolbox&trade; ](https://in.mathworks.com/help/lidar/index.html)

# 3rd Party Products:
Depends on Slamtec RPLIDAR Public SDK for C++ (https://github.com/slamtec/rplidar_sdk). Requires Microsoft Visual Studio &reg; for SDK installation.

# Installation Instructions (Windows x64 Platform)

## Download Slamtec RPLIDAR Public SDK for C++ from Github
Download SDK sources from https://github.com/slamtec/rplidar_sdk and extract the downloaded zip file.
### SDK License
- Slamtec RPLIDAR Public SDK for C++ is licensed under [BSD 2-Clause LICENSE](https://github.com/Slamtec/rplidar_sdk/blob/master/LICENSE)

## Install the required Device Driver Software (If not already installed)
Device driver software for Windows is included in the downloaded SDK.

Navigate to 'tools\cp2102_driver\' folder of the extracted RPLIDAR SDK sources folder.

Extract CP210x_Windows_Drivers zip file and install the driver software by double clicking appropriate setup file (CP210xVCPInstaller_x64.exe or CP210xVCPInstaller_x86.exe) and follow the instructions on the screen.

## RPLIDAR SDK Installation:
Refer to installation instructions found here: https://github.com/Slamtec/rplidar_sdk#on-windows.
Only 'rplidar_driver' project solution needs to be built for this RPLIDAR Streaming package.
Change Build configuration:
- In Visual Studio, change Solution Configuration and Solution platform to 'Release' and 'x64' respectively.
- To do this, right-click on 'rplidar_driver' solution in 'Solution Explorer', select 'Properties' and click on 'Configuration Manager' on top-right corner.
- Create a new x64 Solution platform (if it does not exist) from platform dropdown against 'rplidar_driver' row.  In New Project Platform dialog select copy from Win32 and click OK. If you encounter 'This platform could not be created because a solution platform of the same name already exists' error, uncheck 'Create new solution platforms' option in 'New Project Platform' dialog.
- In Configuration Manager dialog, select 'Release', 'x64' and checkbox in Configuration, Platform and Build entries respectively against 'rplidar_driver' row and close the dialog box. Click OK in Properties page to apply changes.
- Build 'rplidar_driver' solution. Right click on 'rpildar_driver' project in 'Solution Explorer' and click Build.

## Compile mex file
- Open MATLAB
- CD to folder containing this package (slamtec_rplidar_streamer) contents.
- Run compile.m file in the current folder:
```bash
% Select appropriate include and library paths:
>> includePath = '.....\rplidar_sdk-master\sdk\include'; 
>> libraryPath = '.....\rplidar_sdk-master\workspaces\vc14\x64\Release';
>> compile(includePath, libraryPath)
```
## API help and example
Run the following command for help on MATLAB API:
```bash
>> help slamteclidar
<Change the port and baudrates accordingly in the example code>
Run the example to see the sensor stream data.
```
## Demo script
Run the attached demo script 'demo_slamteclidar.m' that plots lidar data in pcplayer as point cloud data.
```bash
>> demo_slamteclidar
```

## Community Support
[MATLAB Central](https://www.mathworks.com/matlabcentral)

Copyright 2023 The MathWorks, Inc.
