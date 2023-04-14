% slamteclidar Creates Slamtec RPLIDAR sensor streamer object
%
% obj = slamteclidar(port, baudrate) Creates Slamtec RPLIDAR lidar
% streamer object that enables reading lidar sensor measurements and
% configure sensor parameters such as motor RPM and scan mode. For valid
% port values, see the output of 'serialportlist' command in MATLAB
% command prompt. Typical values for baudrate are 57600, 115200, 256000,
% 1000000, etc. Every sensor type works for specific baudrate values, for
% instance baudrate value of 115200 works for A1M8 model and baudrate value
% of 1000000 works for S2 model. Please refer to the sensor manual for the
% correct baudrate value.
%
% slamteclidar properties:
%   Port                - Port of the connected RPLIDAR sensor
%   Baudrate            - Baudrate of the connected sensor
%   Model               - Model of the connected sensor
%   FirmwareVersion     - Firmware version of the connected sensor
%   HardwareVersion     - Hardware version of the connected sensor
%   SerialNumber        - Serial number of the connected sensor
%   RPM                 - Vector containing desired, minimum and maximum
%                         motor speeds of the connected sensor
%   SupportedScanModes  - List of scan modes supported by the connected
%                         sensor
%   TypicalScanMode     - Typical (or recommended) scan mode for the
%                         connected sensor
%   ScanMode            - Scan mode in use by the connected sensor
%
%
% slamteclidar methods:
%   start        - Start the sensor scanning for measurements
%   stop         - Stop the sensor scanning
%   read         - Read sensor measurement data
%   setRPM       - Set the motor RPM or speed of the sensor
%   setScanMode  - Set the scan mode of the sensor
%   reset        - Reset the connected sensor or device
%   isConnected  - Check if the sensor is connected
%   isScanning   - Check if the sensor is scanning
%   delete       - Disconnect the sensor and delete resources
%
% NOTES:
% ------
% - delete method MUST be invoked on the slamteclidar object when the
% object is no longer needed for streaming data. Use delete method before
% clearing the object from the workspace. This ensures the connection to
% the device is properly terminated. Not doing so will not close the
% connection to the sensor(or device) and may prevent creating another
% slamteclidar object in the current MATLAB session.
%
% - Only one slamteclidar object can be created at a time. Delete the
% existing object before creating another object.
%
% - read method returns the current or latest scan from the connected
% sensor and previous scans are not stored or buffered.
%
% Example: Create Slamtec RPLIDAR sensor streamer object and plot data
% --------------------------------------------------------------------
% % Create Slamtec RPLIDAR streamer object
% % <Change Port and Baud rate values accordingly>
% slobj = slamteclidar("COM3", 115200);
%
% % Start scanning
% slobj.start();
%
% % Read 10 scans
% for i = 1 : 10
%   scan = slobj.read();
%   plot(scan);
%   pause(0.1);
% end
%
% % Stop scanning
% slobj.stop();
%
% % Close connection and destroy resources
% slobj.delete();
% clear slobj;
%
% See also lidarScan, serialportlist

% Copyright 2023 The MathWorks, Inc.


classdef slamteclidar < handle

    properties (SetAccess = private)
        % String representing serial port of the connected RPLIDAR sensor
        Port

        % Positive integer representing baudrate of the connected sensor
        Baudrate

        % Positive number representing the model of the connected sensor
        Model;

        % String represent firmware version of the connected sensor
        FirmwareVersion;

        % Number representing hardware version of the connected sensor
        HardwareVersion;

        % String representing serial Number of the connected sensor
        SerialNumber;

        % A three element numeric vector representing desired
        % (recommended), minimum and maximum possible motor speeds of the
        % connected sensor
        RPM;

        % String vector representing list of scan modes supported by the
        % connected sensor.
        SupportedScanModes;

        % String representing typical (or recommended) scanmode of the
        % connected sensor.
        TypicalScanMode;

        % String representing the scan mode in use by the connected sensor.
        %
        ScanMode;

        % Number representing scan frequency of the current or latest scan
        % in Hz
        Frequency;
    end

    properties (Access = private, Hidden)
        IsConnected = false;
        IsScanning = false;
    end

    methods
        function this = slamteclidar(port, baudrate)

            arguments
                port {validatePort(port)}
                baudrate {mustBeNonmissing, mustBeScalarOrEmpty, mustBeNonempty, mustBePositive, mustBeInteger, mustBeReal, mustBeNonsparse}
            end

            this.Port = string(port);
            this.Baudrate = baudrate;

            connect(this);
        end

        function status = isConnected(this)
            status = this.IsConnected;
        end

        function status = isScanning(this)
            status = this.IsScanning;
        end

        function start(this)
            %start Start scanning the connected sensor for measurements.

            if(this.IsScanning)
                return;
            end
            this.IsScanning = false;
            try
                mex_rplidar_sdk_interface("startscanning");
            catch ME
                throw(ME);
            end
            this.IsScanning = true;
        end

        function stop(this)
            %stop Stop scanning the connected sensor for measurements

            if(~this.IsScanning)
                return;
            end

            try
                mex_rplidar_sdk_interface("stopscanning");
            catch ME
                throw(ME);
            end
            this.IsScanning = false;
        end

        function reset(this)
            %reset Resets the connected sensor (or device)

            if(this.IsScanning)
                error('Cannot reset device while scanning. Stop the device and try reset again.');
            end
            try
                mex_rplidar_sdk_interface("reset");
            catch ME
                throw(ME);
            end
        end

        function [scan, readings] = read(this)
            %read Read scan data from the connected Slamtec RPLIDAR sensor.
            %
            %  scan = read(slobj) Returns measurement data of the
            %  connected sensor's current (or latest) scan as a lidarScan
            %  object.
            %
            %  [~, readings] = read(slobj) Returns the raw measurement data
            %  of the current scan as 3xN matrix, where N is the number of
            %  measurements. Rows 1, 2 and 3 contain distance (in meters),
            %  azimuth (in radians) and quality data of each measurement
            %  respectively.
            %
            % NOTES:
            % - read() returns the current or latest scan from the
            % connected sensor and previous scans are not stored or
            % buffered.
            %
            % See also lidarScan

            if(~this.IsScanning)
                error('Start the device to read scan data.')
            end
            try
                [readings, freq] = mex_rplidar_sdk_interface("scan");
            catch ME
                throw(ME);
            end
            if(~isempty(readings))
                scan = lidarScan(readings(1,:), readings(2,:));
                this.Frequency = freq;
            else
                error('Error reading scan data from device.');
            end
        end

        function setRPM(this, rpm)
            %setRPM Set the motor RPM or speed of the connected sensor
            %
            %  setRPM(slobj, rpm) Attempts to set the motor speed of the
            %  connected sensor to rpm.
            arguments
                this slamteclidar
                rpm {valdateRPM(rpm, this)}
            end
            if(this.IsScanning)
                error('Cannot set device RPM while scanning. Stop the device and try again.');
            end
            try
                mex_rplidar_sdk_interface("setmotorspeed", rpm);
            catch ME
                throw(ME);
            end
        end

        function setScanMode(this, scanMode)
            %setScanMode Sets the scan mode of the connected sensor.
            %
            %  setScanMode(slobj, scanMode) Attempts to set the scan mode of
            %  the connected sensor to scanMode.

            arguments
                this slamteclidar
                scanMode {valdateScanMode(scanMode, this)}
            end

            scanModeId = find(lower(string(scanMode)) == lower(this.SupportedScanModes)) - 1;
            if(this.IsScanning)
                error('Cannot set device scan mode while scanning. Stop the device and try again.');
            end
            try
                mex_rplidar_sdk_interface("setscanmode", scanModeId);
            catch ME
                throw(ME);
            end

            this.ScanMode = string(scanMode);
        end
    end

    methods(Access = public)
        function delete(this)
            %delete Disconnect the connected sensor.
            disconnect(this);
        end
    end

    methods(Access = 'private')
        function connect(this)
            this.IsConnected = false;
            try
                mex_rplidar_sdk_interface("close");
                clear mex_rplidar_sdk_interface;
                pause(0.1);
                outs = mex_rplidar_sdk_interface("open", this.Port, this.Baudrate);

            catch ME
                disconnect(this); % close connection
                pause(0.1);
                throw(ME);
            end
            pause(0.2);
            this.Model = outs.Model;
            this.HardwareVersion = outs.HardwareVersion;
            this.FirmwareVersion = string(outs.FirmwareVersion);
            this.SerialNumber = string(outs.SerialNumber);
            this.RPM = outs.RPM;
            this.TypicalScanMode = string(outs.TypicalScanMode);
            this.SupportedScanModes = outs.SupportedScanModes;
            this.ScanMode = this.TypicalScanMode;
            this.IsConnected = true;
        end

        function disconnect(this)
            try
                mex_rplidar_sdk_interface("close")
                clear mex_rplidar_sdk_interface;
            catch ME
                throw(ME);
            end
            this.IsConnected = false;
        end
    end

end

% Validation functions
function validatePort(port)
mustBeMember(port, serialportlist())
end

function valdateScanMode(scanMode, obj)
mustBeMember(scanMode, obj.SupportedScanModes);
end

function valdateRPM(rpm, obj)
mustBeInRange(rpm, obj.RPM(2), obj.RPM(3));
end
