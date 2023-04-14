function compile(includePath, libraryPath)
% Helper function to compile mex file

% Copyright 2023 The MathWorks, Inc.

ipath = ['-I' includePath];
lpath = ['-L' libraryPath];
lname = '-lrplidar_driver';

mex_cpp = 'mex_rplidar_sdk_interface.cpp';

cflags = "COMPFLAGS=$COMPFLAGS /MT";
mex(mex_cpp, cflags, ipath, lpath, lname)
end
