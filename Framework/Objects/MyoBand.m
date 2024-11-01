classdef MyoBand < handle
    
    properties (GetAccess = 'public', SetAccess = 'public')
        %% AcqSysInterface properties
        % Basic info
        devName = 'myoband';        % Name of the acquisition system
        devType = 'EMG';        % Name of the variable being measured,i.e. EMG / force / kinematics
        dataType = 'int8';      % Bit precision of the received data
        sampleRate = 200;       % Sampling frequency [Hz]
        nCh = 8;                % Total number of channels
%         AuxCh = 2;              % Total number of auxiliary channels
%         StiCh = 6;
%         nDofs = 4;
        % Acquisition
        int2mV = 1;%2^-7;          % Integer to mV conversion
        mV2int = 1;             % mV to integer conversion
        AcqMod = 'bipolar'      % Acquisition mode
        
        % Buffers
        acqBuffer;          % FIFO buffer for data acquisition
        acqBufferTime = 1;         % Size in [s] of the FIFO buffer for acquisition
        acqBufferSize           % Size in [samples] of the FIFO buffer for acquisition
        recBuffer = [];         % Buffer for data recording (data is infinitely stacked to avoid information loss)
        recBufferSize = 10240;  % Size in [s] of the buffer data recording if known, otherwise it will keep stacking information
        
        % Flags
        isConnected = false;    % Connection flag
        isAcquiring = false;    % Acquisition flag
        isRecording = false;    % Recording flag
        isEventBased = false;   % Execution flag (event based or not)
        
        % Clocks
        ClockStartAcq           % Clock at the start of acquisition
        ClockStartRec           % Clock at the start of recording
        ClockStopAcq            % Clock at the stop of acquisition
        ClockStopRec            % Clock at the stop of recording
        
        % Other properties
        otherProp = false;      % Specific properties to be saved on recinfo
        
        %% Specific properties
        % Timer period for the buffer
        timerPeriod=0.04;
        % Communication
        port                    % Port for TCP/IP communication
        con                     % Connection fid
        channels                % Array with the recording channels
        
        % Recording
        fid                     % Fid of the recording
        savePath             % Path and file name to store the data during recording
        saveBinPath
        % Additional flags
        autoStart = false;      % Flag to automatically start MyoTcpBridge.exe 
        
%         %Variables for Serial object
%         EMGSerial;
%         byteCount
        % Data properties
        new_data = [];
        RecInfo;
    end
    
    methods(Access = public)
        
        %% Constructor
        function obj = MyoBand(port)
            % consturctur: initializes everything and establishes TCP/IP connection
            % to MyoBilateralConnector.
            % Start Myo Connect before, connect to both MYOs and run
            % MyoBilateralConnector.exe before!
            nmyos = 1;
            % Port
            if nargin < 3
                nmyos = 1;
                obj.port = 3333;
                %disp('using port 3333 for connection with myoConnect');
            else
                obj.port = port;
            end
            
            % Acquisition block
            %obj.acqBufferTime = 0.04;   % [s] - Duration of each reading package (same as execution timer period)
            obj.acqBufferSize = floor(0.04*obj.sampleRate);   % Standard block size (number of samples received in each getData call)
            
            % Flags
            obj.autoStart = true;  % Flag to automatically start connector program
            obj.isConnected = false;
            
            % Automatically runs Myo TCP Bridge
            if obj.autoStart
                % search if an old connector is allready running:
                %[error,message] = dos('TASKLIST /FI "imagename eq MyoConnectorUniversal.exe" /svc');
                
                % kill any potentially running old connector:
                [error_id, message] = dos('taskkill /F /IM MyoTcpBridge.exe');
                if ~error_id % no error -> there was an old connector running
                    disp('Found and killed an old MyoConnector');
                end
                
                % start new myoConnector:
                disp('Trying to start new MyoConnector, check the cmd window for details');
                [error_id,message] = dos(['MyoTcpBridge.exe' num2str(obj.port) ' &']);   
                fullfilePath=which('MyoTcpBridge.exe');
                system([fullfilePath ' ' num2str(obj.port)  ' &']);
            end
            
            pause(0.3);
            
            obj.con = pnet('tcpconnect','localhost',obj.port);
            
            % Number of Myos
            if nmyos == 1
                obj.channels = 1:8;
                pnet(obj.con,'write', '1myo' );
                pause(0.1);
            elseif nmyos == 2
                obj.channels = 1:16;
                pnet(obj.con,'write', '2myo' );
            end
            obj.nCh = length(obj.channels);
            %write(obj.EMGSerial, 'p', 'char');
            obj.acqBuffer = zeros(obj.nCh, obj.sampleRate * obj.acqBufferTime);
            
            pause(0.05)
            fprintf('Connected to device!')
            
        end
        
        %% Destructor
        function delete(obj)
            try
                if obj.isConnected
                    obj.stopAcq();
                end
                
                if obj.autoStart
                    % tell MyoTcpBridge to close:
                    disp('close MyoTcpBridge.exe...');
                    pnet('closeall')
                    % Changed to avoid breaking Matlab when stopping DynDOF framework.
                    % Downside is that it cannot be started again, GUI needs to be
                    % restarted. Otherwise use the original line:
                    % pnet(obj.con, 'write', 'close');
                    pause(0.1);
                end
                
            catch
                disp('error while closing client');
            end
        end
        
        %% Connection
        
        % Communication is established automatically when the object is
        % constructed - startComm is not needed
        function  out = startComm(obj)
            obj.isConnected = true;
            out = true;
        end
        
        function stopComm(obj)
            obj.isConnected = false;
        end
        
        %% Acquisition
        
        function out = startAcq(obj)
            % Starts acquisition. To iteratively collect data call getData
            
            % clear TCP/IP-buffer:
            pnet(obj.con,'setreadtimeout',0);
            pnet(obj.con,'read');% emplty buffer
            
            % tell server to start:
            pnet(obj.con,'write', 'startmyo' );
            pause(0.1)
            
            % empty TCP buffer
            newdata = nan;
            while ~(isempty(newdata))
                newdata = pnet(obj.con,'read',[obj.nCh obj.acqBufferSize] ,obj.dataType);
            end
           
            obj.ClockStartAcq = clock;
            
            obj.isConnected = true;
            obj.isAcquiring = true;
            out = true;
        end
        
        function out = stopAcq(obj)
            % Stops acquisition
            
            % Stop MyoTcpBridge
            pnet(obj.con,'write', 'stopmyo' );
            pnet(obj.con,'setreadtimeout',inf);
            
            obj.ClockStopAcq = clock;
            
            obj.isConnected = false;
            obj.isAcquiring = false;
            out = true;
        end
        
        %% Recording
        
        function out = startRec(obj, path)
            % Saving path
             if isempty(path)
                obj.savePath = pwd;
                obj.saveBinPath=fullfile(obj.savePath, ['rawdata_' obj.devName '.bin']);
            else
                obj.savePath = path;
                obj.saveBinPath=fullfile(obj.savePath, ['rawdata_' obj.devName '.bin']);
             end
            
            % FID for binary recording
            obj.fid = fopen(obj.savePath,'w');
            if obj.fid > 0
                obj.isRecording = true;
            else
                obj.isRecording = false;
            end
            
            % Other properties to track samples
            obj.otherProp = struct('batch_num',0,'batch_size',[]);
                        
            % Update flags
            obj.ClockStartRec = clock;
            out = obj.isRecording;
        end
        
        function out = stopRec(obj,RecInfo)
            
            obj.isRecording = false;
            obj.ClockStopRec = clock;
            
            EMGraw=readBin_simple([obj.nCh,Inf],'int16',obj.savePath);
            RecInfo.EMGraw=EMGraw';
            
            save(obj.savePath,'RecInfo','-v7.3')
            fclose(obj.fid);
            
            out = obj.isRecording;
        end
        
        
        %% Get data
        
        function getData(obj)
            % get all data from MyoTcpBridge since last call
            
            if obj.isConnected
                newdata = [];
                while obj.isConnected && isempty(newdata) % collect all TCP-packets that are in buffer
                    newdata = pnet(obj.con,'read',[obj.nCh obj.acqBufferSize] ,obj.dataType);
                    
                    if isempty(newdata)
                        newdata = (zeros(obj.nCh, obj.acqBufferSize));
                    end
                    newdata = fliplr(newdata);
                    obj.new_data = double(newdata)*obj.int2mV;
                    
                    if obj.isRecording                                                                                                                              
                        fwrite(obj.fid, newdata, obj.dataType);                                                                                                     
                        obj.otherProp.batch_num = obj.otherProp.batch_num + 1;                                                                                      
                        obj.otherProp.batch_size = [obj.otherProp.batch_size size(newdata,2)];                                                                                
                    end
            
                    %obj.acqBuffer = [obj.acqBuffer obj.new_data];
                    obj.acqBuffer = circshift(obj.acqBuffer,[0,-obj.acqBufferSize]);
                    obj.acqBuffer(:, end - obj.acqBufferSize + 1:end) = obj.new_data;
                end
            else
                obj.acqBuffer = zeros(obj.nCh, obj.sampleRate * obj.acqBufferTime);
                warning('Acquisition has not started')
            end
            
        end
    end
    
end

