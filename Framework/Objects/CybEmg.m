classdef CybEmg < handle
    
    properties (GetAccess = 'public', SetAccess = 'public')
        %% AcqSysInterface properties
        % Basic info
        devName = 'ArmIO';     % Name of the acquisition system
        devType = 'EMG';        % Name of the variable being measured,i.e. EMG / force / kinematics
        dataType = 'int16';    % Bit precision of the received data
         % Properties of the device
        devMode     = 'Master';       % If master or slave. 1 - Master, 2 - Slave
        isSlaveAvailable = 0;
        sampleRate  = 1000;      % Sampling frequency [Hz]
        nCh         = 12;               % Total number of channels by default
        AuxCh       = 2;              % Total number of auxiliary channels
        nDofs       = 4;
        NormChs     = 14;
        RmsChs      = 14;
        StiCh       = 6;
        kinCh       = 6;
        isSDModule  = 0;
        isBluetooth = 0;
        
        isDisplayCommand = false % Display the cmd 
        
        % Acquisition
        int2mV = 1;%3.3/2^12;      % Integer to mV conversion
        mV2int = 1;             % mV to integer conversion
        AcqMod = 'bipolar'      % Acquisition mode
        
        % Buffers
        acqBuffer;         % FIFO buffer for data acquisition
        acqBufferTime = 1;      % Size in [s] of the FIFO buffer for acquisition
        acqBufferSize           % Size in [samples] of the FIFO buffer for acquisition
        
       % Flags
        isConnected = false;    % Connection flag
        isAcquiring = false;    % Acquisition flag
        isRecording = false;    % Recording flag
        isEventBased = true;   % Execution flag (event based or not)
        
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
        
        % Recording
        fid                     % Fid of the recording
        savePath             % Path and file name to store the data during recording
        saveBinPath
        
        % Variables for Serial object
        EMGSerial;
        byteCount
        Haptic = zeros(1, 6);
        
        % Data properties
        new_data=[];
        RecInfo;
        
        % COM properties
        HandSequence = [1, 2, 3]; 
        isCheckSum = 1;
        isHandShake = 1;
        
    end
    
    methods(Access = public)
        %% Constructor
        function obj = CybEmg()
            %!python3 SocketV4.py &
            switch nargin
                case 0
                    prompt = {'Enter the serial Port'};
                    dlgtitle = 'Serial Port';
                    dims = [1 35];
                    definput = {'ArmIO-a302'};
                    Port = inputdlg(prompt,dlgtitle,dims,definput,'on');
                    Port=Port{1};
            end
            % Define properties of the device
            %obj.acqBufferSize = floor(obj.timerPeriod*obj.sampleRate); 
            %baudRate = 115200;
            %baudRate = 2000000;

            % Create serial object
            fprintf('Attempting connection...\n');
            obj.EMGSerial = bluetooth(Port, 1);
            
            % Get the device properties
            fprintf('Reading device settings...\n')
            write(obj.EMGSerial, 'p', 'char');  % Stop if transmitting any data
            pause(0.05);
            obj.EMGSerial.flush;
            write(obj.EMGSerial, 'c', 'char');  % Read the properties of the device
            pause(0.05);

            
            % Read properties of the device
            isMaster           = read(obj.EMGSerial, 1, 'int16'); 
            if (isMaster)
                obj.devMode = 'MASTER'; 
            else  
                obj.devMode = 'SLAVE';
            end
            obj.isSlaveAvailable = read(obj.EMGSerial, 1, 'int16'); 
            obj.sampleRate    = read(obj.EMGSerial, 1, 'int16'); 
            obj.acqBufferSize = read(obj.EMGSerial, 1, 'int16'); 
            obj.nCh           = read(obj.EMGSerial, 1, 'int16'); 
            obj.AuxCh         = read(obj.EMGSerial, 1, 'int16'); 
            obj.nDofs         = read(obj.EMGSerial, 1, 'int16'); 
            obj.NormChs       = read(obj.EMGSerial, 1, 'int16');
            obj.RmsChs        = read(obj.EMGSerial, 1, 'int16'); 
            obj.StiCh         = read(obj.EMGSerial, 1, 'int16'); 
            obj.kinCh         = read(obj.EMGSerial, 1, 'int16'); 
            obj.isSDModule    = read(obj.EMGSerial, 1, 'int16');                 
            obj.isHandShake   = read(obj.EMGSerial, 1, 'int16');                 
            obj.isCheckSum    = read(obj.EMGSerial, 1, 'int16');                 
            obj.EMGSerial.flush;

            % Generate callback
            % Create function handle
            
            callbackFun = @obj.getData;
            EMG_bufferBytes = obj.nCh * 2 * obj.acqBufferSize; 
            AUX_bufferBytes = obj.AuxCh * 2 * obj.acqBufferSize;
            Reg_bufferBytes = obj.nDofs * 2;
            Norm_bufferBytes = obj.NormChs * 2;
            rms_bufferBytes = obj.RmsChs * 2 ;
            Haptic_bufferBytes = obj.StiCh * 2 ;
            Motion_bufferBytes = obj.kinCh * 2;
            checkSum_bytes = 0;
            if obj.isCheckSum
                checkSum_bytes = checkSum_bytes + 2;
            end
            if obj.isHandShake
                checkSum_bytes = checkSum_bytes + 3 * 2;
            end


            % Determine the package size to read when device is in
            % transmission
            obj.byteCount = EMG_bufferBytes + AUX_bufferBytes + ...
                            Reg_bufferBytes + Norm_bufferBytes + ...
                            Haptic_bufferBytes + rms_bufferBytes + Motion_bufferBytes + checkSum_bytes; 
            configureCallback(obj.EMGSerial, "byte", obj.byteCount, callbackFun);

            %obj.EMGSerial.Timeout=2;
            
            obj.acqBuffer = zeros(obj.nCh, obj.sampleRate * obj.acqBufferTime);
            %obj.AuxData = zeros(2, obj.acqBufferSize);
            
            fprintf('Connected to device!')
        end
        
        %% Update
        function update(obj, ch, fs, period)
            obj.nCh = ch;
            obj.sampleRate = fs;
            obj.timerPeriod = period;
            %obj.acqBufferSize = floor(obj.timerPeriod*obj.sampleRate); 
            obj.acqBufferSize = floor(40); 
        end
                
        %% Destructor
        function delete(obj)
            if obj.isConnected
                obj.stopAcq();
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
            write(obj.EMGSerial, 'p', 'char');  % Stop if transmitting any data
            obj.EMGSerial.delete;
            obj.isConnected = false;
            fprintf('Disconnected.\n');
        end
        
        %% Acquisition
        
        function out = startAcq(obj)
            % Starts acquisition
            obj.isConnected = true;
            obj.isAcquiring = true;
            obj.ClockStartAcq = clock;
            out = true;
            
            % Send command to socket
            write(obj.EMGSerial, 'p', 'char');
            obj.EMGSerial.flush;
            write(obj.EMGSerial, 's', 'char');
        end
        
        function out = stopAcq(obj)
            % Send command to socket
            write(obj.EMGSerial, 'p', 'char');
            obj.EMGSerial.flush;
            
            % Stops acquisition acquisition
            obj.ClockStopAcq = clock;
            obj.isAcquiring = false;
            obj.isConnected = false;
            out = true;
        end
        
        %% Get data
        function getData(obj, varargin)
            StartStatus = 0;
            CS_status = 0;
            
            if obj.isHandShake
               StartStatus = obj.Handshake();
            else
                StartStatus = 1;
            end
                
            if StartStatus
                % Get data from the socket
                for i = 1: obj.nCh
                    obj.new_data(i,:) = read(obj.EMGSerial, obj.acqBufferSize, obj.dataType);
                end
                obj.new_data = fliplr(obj.new_data);

                % Read Aux ADC ports
                if obj.AuxCh > 0 
                    for i = 1:obj.AuxCh
                        AuxData(i,:)=read(obj.EMGSerial, obj.acqBufferSize, obj.dataType);
                    end
                    AuxData = fliplr(AuxData);
                end
                % extract regressor values
                if obj.nDofs > 0
                    Reg = read(obj.EMGSerial, obj.nDofs, obj.dataType);
                end
                % Extract norm EMG values
                if obj.NormChs > 0 
                    Norm = read(obj.EMGSerial, (obj.nCh + obj.AuxCh), obj.dataType); % Check with MCU
                end
                % Extract rms EMG values
                if obj.RmsChs > 0
                    Rms = read(obj.EMGSerial, (obj.nCh + obj.AuxCh), obj.dataType);
                end
                % Extract haptic values
                if obj.StiCh > 0
                    obj.Haptic = read(obj.EMGSerial, (obj.StiCh), obj.dataType);%./255.*100; [1 x nMotors]
                end
                % Read kinematics
                if obj.kinCh > 0
                    Kinematics = read(obj.EMGSerial, 6, obj.dataType) ./ 4096 * 100;%./255.*100; [1 x nMotors]
                end
            end
            
            if obj.isCheckSum
                %SumStatus = 1;
                checksum = read(obj.EMGSerial, 1, obj.dataType) ;
                % Requires a reshape of the data
                CS_status = obj.Checksum({obj.new_data, AuxData, Reg, Norm, Rms}, checksum);
            else
                CS_status = 1;
            end

            if CS_status
                % Store data if recording
                if obj.isRecording
                    fwrite(obj.fid, obj.new_data, obj.dataType);
                    obj.otherProp.batch_num = obj.otherProp.batch_num + 1;
                    obj.otherProp.batch_size = [obj.otherProp.batch_size size(obj.new_data,2)];
                end

                % acqBuffer - [nchan x length]
                %obj.acqBuffer = [obj.acqBuffer obj.new_data];
                obj.acqBuffer = circshift(obj.acqBuffer,[0,-obj.acqBufferSize]);
                obj.acqBuffer(:, end - obj.acqBufferSize + 1:end) = obj.new_data;
            end
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
        %% Send data to micro
        function SendCmd(obj,cmd)
                write(obj.EMGSerial, cmd, 'string');
        end
        function status = Handshake(obj)
            % Check if block starts with the correct handshake
            sequence = read(obj.EMGSerial, 3, obj.dataType);
            status = 1;

            if sequence(1) == obj.HandSequence(1) && sequence(2) == obj.HandSequence(2) && sequence(3) == obj.HandSequence(3)
                status = 1;
            % If the sequence is wrong 
            else
                isChecking = true;
                % If not check when the next sequence of data block starts
                while isChecking
                    sequence(3) = sequence(2) ;
                    sequence(2) = sequence(1);
                    sequence(1) = read(obj.EMGSerial, 1, obj.dataType);
                    if sequence(1) == obj.HandSequence(3) && sequence(2) == obj.HandSequence(2) && sequence(3) == obj.HandSequence(1)
                        isChecking = false;
                        status = 1;
                        fprintf('wrong block\n')
                    end
                end
            end

        end
        function status = Checksum(obj, data, checksum)
            %%
            status = 0;
            Chks = int16(0);
            div = int16(32766);
            % do calculations
            for i = 1: 40
                for j = 1 : 12 %obj.nCh
                    Chks = rem(Chks + (data{1}(j,i)), div);
                end
                %{
                if obj.AuxCh > 0 RecIn
                    for j = 1: obj.AuxCh
                        Chks = rem(Chks + data{2}(j,i), div);
                    end
                end
                %}
            end
            Chks = rem(Chks, div);
  %{
            for i = obj.nDofs : -1 :1
                Chks = rem(Chks + data{3}(i), div);
            end

            for i = 1: obj.NormChs
                Chks = rem(Chks + data{4}(i), div);
                Chks = rem(Chks + data{5}(i), div);
            end
            for i = obj.HapticEngine.nMotors  : -1 :1
                Chks = rem(Chks + data{6}(i), div);
            end
            %}
            %disp(Chks)
            if Chks == checksum
                status = 1;
            else
                fprintf('Wrong block\n');
            end
            
            %%
        end
        

    end
    
end
