function [data] = readBin_simple(size, precision, varargin)

if nargin < 3
    [file,main_path] = uigetfile('*.bin','Select a bin file:');
    path = fullfile(main_path,file);
else
    path = varargin{1};
end

% Open the binary file
fid = fopen(path, 'r');
fseek(fid, 0, 'bof');  

% Read the data
data = fread(fid, size, precision);

% Close file
fclose(fid);
end

