function demo_slamteclidar()
% slmateclidar API demo application

% Copyright 2023 The MathWorks, Inc.

% Change port and baudrate values accordingly
slobj = slamteclidar("COM3", 115200); % sample port and baudrate values for Slamtec RPLIDAR A1 model
% slobj = slamteclidar("COM4", 1000000); % sample port and baudrate values for Slatec RPLIDAR S2 model

cleanupObj = onCleanup(@()slobj.delete);

if(~slobj.isConnected())
    return;
end

% Create pcplayer object
player = pcplayer([-20 20], [-20 20], [0 1]);
pause(0.1);

% Change scan mode to 'Standard'
% slobj.setScanMode("Standard");

% Update RPM to average of min and max RPMs.
% avgRPM = floor(mean(slobj.RPM(2:3)));
% slobj.setRPM(avgRPM);

% Start scanning
slobj.start();

% Read and plot scan data as long as the player is open
while(slobj.isScanning() && player.isOpen())
    % Read scan data
    scan = slobj.read();
    
    if(~isempty(scan))
        % Create pointCloud object and update pcplayer
        view(player, pointCloud([scan.Cartesian zeros(size(scan.Cartesian,1), 1)]));
    end
    pause(0.15);
end

% Stop scanning
slobj.stop();

% delete and clear object
slobj.delete();
clear slobj;

end


