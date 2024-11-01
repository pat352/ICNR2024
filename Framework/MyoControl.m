classdef MyoControl < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                    matlab.ui.Figure
        DAQPanel                    matlab.ui.container.Panel
        PortDropDownLabel           matlab.ui.control.Label
        dp_Port                     matlab.ui.control.DropDown
        cb_isConnected              matlab.ui.control.CheckBox
        b_Acq                       matlab.ui.control.Button
        Dev                         matlab.ui.control.DropDown
        isPlotEMG                   matlab.ui.control.CheckBox
        TrainingPanel               matlab.ui.container.Panel
        b_recFolder                 matlab.ui.control.Button
        t_folder                    matlab.ui.control.EditField
        GestureDropDownLabel        matlab.ui.control.Label
        dp_Gesture                  matlab.ui.control.DropDown
        b_cues                      matlab.ui.control.Button
        RegressorPanel              matlab.ui.container.Panel
        b_Reg                       matlab.ui.control.Button
        t_reg                       matlab.ui.control.EditField
        b_Train                     matlab.ui.control.Button
        b_Test                      matlab.ui.control.Button
        HandPanel                   matlab.ui.container.Panel
        cb_isHand                   matlab.ui.control.CheckBox
        PortDropDownLabel_2         matlab.ui.control.Label
        dp_HandPort                 matlab.ui.control.DropDown
        SmoothingFIlterFreqHzLabel  matlab.ui.control.Label
        FilterFreq                  matlab.ui.control.Knob
        ModelBasedPanel             matlab.ui.container.Panel
        cb_EnableRob                matlab.ui.control.CheckBox
        b_Create                    matlab.ui.control.Button
        GainKnobLabel               matlab.ui.control.Label
        Gain                        matlab.ui.control.Knob
        ConstantExternalTorqueKnobLabel  matlab.ui.control.Label
        ExtTorque                   matlab.ui.control.Knob
        ExtensorChLabel             matlab.ui.control.Label
        ed_ExtensorCh               matlab.ui.control.NumericEditField
        FlexorChLabel               matlab.ui.control.Label
        ed_FlexorCh                 matlab.ui.control.NumericEditField
        UIAxes                      matlab.ui.control.UIAxes
    end

    
    properties (Access = private)
        % System
        ExeTimer              % GUI execution cycle
        DAQDevice             % Acquisition system object
        isConnected           % Flag for device connection
        isTrained = 0;          % Flag to indicate if system have a trained regressor
        isRecTrainData = 0;    % Flag to indicate if system is recording data
        
        % Recordings
        recCounter = 1;       % Counter for the recordings
        recPath;              % Actual recording path
        recName;              % Recording name for the training data
        RecInfo;              % Recording information for the training data
        feat2test;
        
        % EMG Axes properties
        PlotEMG;               % Axes for EMG plotting
        PlotOffset = 1500;     % Offset distance between EMG plotting channels
        PlotOffsets;           % Offset for each primitive EMG line
        lemg;                  % Primitive lines for EMG
        TimeWindowToPlot = 1;  % Time window to plot
        xAxisTime;
        
        % cursor properties
        Cursor;                % Cursor for virtual enviroment
        LabelsRec;             % Pre-recorder cursor gestures
        GestureLabel = 0;     % Pre-recorded cursor gesture to record and plot 
        CursorSample = 1;      % Current cursor sample
        
        % Hand properties
        isHandConnected = 0;
        HandCom;
        nDofs = 2;
        
        % Training properties
        RMSWindow = 160;        % RMS window 
        EMGRms;                 % Calculate EMG rms to save during recording
        RegCoef = 0;            % Regressor matrix coeficcients
        softCtrl;
        
        myrobot;
    end
    
    
    methods (Access = public)
        % Main loop
        function ExeCycle(varargin)
            app = varargin{1};
            app.RMSWindow = 250;
            
            try 
                if strcmp(app.Dev.Value, 'MyoBand')
                    app.DAQDevice.getData();
                end
                % If system is acquiring data
                if app.DAQDevice.isAcquiring
                    
                    % A. EMG PLOTTING -------------------------------------
                    if app.isPlotEMG.Value
                        % Plotting EMG
                        if length(app.DAQDevice.acqBuffer) == (app.DAQDevice.sampleRate * app.TimeWindowToPlot)
                            % Select the data to plot from the data
                            DataToPlot=app.DAQDevice.acqBuffer(:,end-((app.DAQDevice.sampleRate * app.TimeWindowToPlot)-1):end);
                            app.PlotChannels(app.DAQDevice.nCh,DataToPlot);
                        end
                    end
                    
                    % B. REAL TIME  -------------------------------
                    if size(app.DAQDevice.acqBuffer,2)  > app.RMSWindow
                        
                        % 1. New block of data to process RMS of 160 samples
                        DataRmsWindow = app.DAQDevice.acqBuffer( : , end - app.RMSWindow + 1:end);
                        
                        % 2. Feature extraction of the RMS
                        newRMS      = T1A_ExtractRms(DataRmsWindow');
                        app.EMGRms  = [app.EMGRms; newRMS];

                        % Get the estimation if regression coeficient are loaded
                        if app.isTrained
                            
                            % 3. Compute the estimation output with the regressor
                            Estimate = T1C_TestReg([1, newRMS], app.RegCoef); 
                            for i = 1: 2
                                if (abs(Estimate(i)) < 20); Estimate(i) = 0; end
                            end
                            
                            % 4. Smoothing of the output of the regressor
                            for i = 1: app.nDofs
                                app.softCtrl{i}.inData(Estimate(i))
                                Estimate(i) = app.softCtrl{i}.out;
                            end
                            
                            % 6. Constrains to avid values out of range
                            for i = 1: 2
                                if (Estimate(i) > 100); Estimate(i) = 100; end
                                if (Estimate(i) < -100); Estimate(i) = -100; end
                            end
                            
                            %%%%%%%%% Day3 %%%%%%%%%%%%
                            % Tune the gains of a position controller to track Estimate on a given robot model
                            
                            if app.cb_EnableRob.Value
                                qt = Estimate(2); % target position
                                Kp = app.Gain.Value;%20; % proportional gain
                                u_ext = app.ExtTorque.Value;%0; % constant external torque
                                qout = app.myrobot.positionController(qt,Kp,u_ext); % defined in MyRobot class
                                %qout
                                Estimate(2) = qout; % modifying to visualise the flexion-extension state of the robot
                                % Modify the proportional parameters using your  muscle cocontraction!
                                %emg1 = newRMS(app.ed_FlexorCh.Value);
                                %emg2 = newRMS(app.ed_ExtensorCh.Value);
                                %[qout] = app.myrobot.variablePosController(qt, emg1, emg2, u_ext);
                            end
                            
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%
                            % 7. Send parameters to the hand - if connected
                            if app.isHandConnected
                                cmd = strcat('a',num2str(round(Estimate(2),0)),',',num2str(round(Estimate(1),0)));
                                
                                write(app.HandCom, cmd, 'string');
                                %disp(cmd)
                            end
                            
                            % 8. Plot the estimation output on the cursor
                            app.PlotCursor(Estimate(1),Estimate(2));
                        end
                        
                    end                    
                end
                
                % C. RECORDING DATA FOR TRAINING --------------------------
                if app.isRecTrainData   
                    % Plot the visual gesture clue for the training data
                    app.PlotCursor(app.GestureLabel(1,app.CursorSample),app.GestureLabel(2,app.CursorSample));
                    
                    % If last sample to record
                    if app.CursorSample == length(app.GestureLabel)
                        app.b_cuesButtonPushed();
                    % if not next sample
                    else
                        app.CursorSample = app.CursorSample + 1;
                    end
                end
                
            catch e
                disp('Error message:');
                disp(e.message);
                e.stack(1)
            end
        end
    end
    
    % SYSTEM FUNCTIONS - NOT MODIFY
    methods (Access = public)
        function PlotCursor(app,cRotation,cAperture)
            % Calculate cursor points
            
            if strcmp(app.dp_Gesture.Value,'SupPron') || strcmp(app.dp_Gesture.Value,'OpnClose')
                cRotation = -0.6 * cRotation + 90;
                cAperture = 0.3 * cAperture + 50;
                X = [0,0];
                Y = [0,0];
                Y(2) = cAperture/2 * sind(cRotation);
                X(2) = cAperture/2 * cosd(cRotation);
                Y(1) = -Y(2);
                X(1) = -X(2);
            else
                A=25;
                R=0;
                X = [cAperture,cAperture];
                Y = [cRotation-20,cRotation+20];
            end
            % Update primitive lines
            set(app.Cursor,'XData',[X(1),X(2)],'YData',[Y(1),Y(2)]);
            
        end
        
        function PlotChannels(app,DAQChannels,DataToPlot)
            for nChan = 1:DAQChannels
                ydata_aux = DataToPlot(nChan,:);
                newFz = 800;
                ydata = resample(ydata_aux,newFz,app.DAQDevice.sampleRate);
                app.xAxisTime = linspace(0,1,newFz);
                set(app.lemg{nChan},'XData',app.xAxisTime','YData',ydata + app.PlotOffsets(nChan));
            end
        end
    end

    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            
             % NO MODIFY BY STUDENT ! ---------------------------
            % System variables
            addpath(genpath(fullfile(pwd,'Objects')));
            
             % Load Pron/sup & open/close cues
            % Get training data
            load(fullfile(pwd,'Cues','SupPron.mat'));
            app.LabelsRec{1} = CUES(3:4,:);
            load(fullfile(pwd,'Cues','OpenClose.mat'));
            app.LabelsRec{2} = CUES(3:4,:);
            
            % EMG PLOT
            app.PlotEMG=axes(app.UIFigure,'position',[0.04, 0.06, 0.445, 0.7250]);
            app.PlotEMG.Box='on';
            app.PlotEMG.Toolbar.Visible='off';
            app.PlotEMG.Interruptible='off';
            app.PlotEMG.PickableParts='none';
            app.PlotEMG.HitTest='off';
            app.PlotEMG.Visible='on';
            app.PlotEMG.XLabel.String = 'Time [S]';
            app.PlotEMG.XLabel.FontWeight = 'bold';
            app.PlotEMG.YTickLabel = {};
            app.PlotEMG.XLabel.FontSize = 14;
            
             
            % Get core
            app.ExeTimer = timer( ...
                'Name', 'PlotTimer', ...
                'TimerFcn',{@app.ExeCycle},...
                'Period',0.04,...,
                'StartDelay',0,...
                'StartFcn','', ...
                'StopFcn',              '', ...
                'ExecutionMode','fixedRate',...
                'BusyMode','queue');
            
            % Scan ports
            app.dp_PortValueChanged();
            app.dp_GestureValueChanged();
            app.dp_HandPortValueChanged2();
            
            % Update graphics
            app.Cursor=line(app.UIAxes,'XData',[0,0],'YData',[-25,25],'color',[0,0.8,0.8],'linewidth',5);
            
        end

        % Value changed function: cb_isConnected
        function cb_isConnectedValueChanged(app, event)
            % If connect
            if app.cb_isConnected.Value
                try
                    instrreset
                    switch app.Dev.Value
                        case 'ArmIO'
                            % Create DAQ object
                            %app.DAQDevice = CybEmg(app.dp_Port.Value);
                            app.DAQDevice = CybEmg();
                        case 'MyoBand'
                            % Create DAQ object
                            %app.DAQDevice = CybEmg(app.dp_Port.Value);
                            app.DAQDevice = CybEmg();
                    end
                    
                    % Activate Connection flag
                    app.isConnected = true;
                    
                    % GUI update
                    app.b_Acq.Enable = 'on';
                    app.isPlotEMG.Enable = 'on';
                    app.dp_Port.BackgroundColor = [0.39,0.83,0.07];
                       
                catch e
                    disp(e)
                    app.cb_isConnected.Value = 0;
                    app.dp_Port.BackgroundColor = [1.00,0.00,0.00];
                end
            % If Disconnect
            else
                % check if device is acquiring
                if app.DAQDevice.isAcquiring
                    app.b_AcqPushed();
                    % Update button 
                    %app.b_Acq.Text = 'Start Acquisition';
                end
                
                % Desactivate connection flag
                app.isConnected = false;
                

                % Stop communication and delete DAQ object
                app.DAQDevice.stopComm();
                delete(app.DAQDevice);
                
                % Enable buttons
                app.b_Acq.Enable = 'off';
                app.isPlotEMG.Enable = 'off';
                app.dp_Port.BackgroundColor = [1.00,0.41,0.16];
            end
        end

        % Callback function: dp_Port, dp_Port
        function dp_PortValueChanged(app, event)
            % NO MODIFY BY STUDENT ! ---------------------------
            % Scan COM ports on the system
            %{
            if ismac
                [stat,result] = system('ls /dev/cu.*');
                start_indx = strfind(result,'/dev/');
                    for i = 1:size(start_indx,2)
                        if i==size(start_indx,2)
                            DevAvailable{i}=result(start_indx(i):end);
                            DevAvailable{i}=DevAvailable{i}(find(~isspace(DevAvailable{i})));
                        else
                            index_end=strfind(result( start_indx(i):end ),'/');
                            DevAvailable{i}=result(start_indx(i):start_indx(i)+index_end(3));
                            option_end=strfind(DevAvailable{i},'	');
                            if ~isempty(option_end)
                                DevAvailable{i}=DevAvailable{i}(1:option_end-1);
                            else 
                                option_end=regexp(DevAvailable{i}, '[\n]');
                                DevAvailable{i}=DevAvailable{i}(1:option_end-1);
                                DevAvailable{i}=DevAvailable{i}(find(~isspace(DevAvailable{i})));
                            end
                        end
                    end
            end
            %}
            %if ispc
                DevAvailable = serialportlist;
            %end
            app.dp_Port.Items = DevAvailable;
        end

        % Button pushed function: b_Acq
        function b_AcqPushed(app, event)
            % NO MODIFY BY STUDENT ! ---------------------------
            if app.isConnected
                % if not acquiring, start to acquire
                if ~app.DAQDevice.isAcquiring
                    app.DAQDevice.startAcq();
                    % Enable recording button
                    app.dp_Gesture.Enable = 'on';
                    app.b_Reg.Enable = 'on';
                    app.b_cues.Enable = 'on';
                    % Update button 
                    app.b_Acq.Text = 'Stop Acquisition';
                    app.b_Acq.BackgroundColor = [1.00,0.41,0.16];
                    
                    
                    % Start the timer
                    start(app.ExeTimer);
                else
                    % Stop timers
                    stop(app.ExeTimer);
                
                    % Stop acquisition
                    app.DAQDevice.stopAcq();
                    
                    % Update GUI 
                    app.b_Acq.Text = 'Start Acquisition';
                    app.dp_Gesture.Enable = 'on';
                    app.b_Reg.Enable = 'off';
                    app.b_cues.Enable = 'off';
                    app.b_Acq.BackgroundColor = [0.39,0.83,0.07];
                    
                end
            end
        end

        % Button pushed function: b_cues
        function b_cuesButtonPushed(app, event)
            if app.isConnected && app.DAQDevice.isAcquiring
                % if not acquiring, start to acquire
                if ~app.DAQDevice.isRecording
                    %app.recPath = fullfile(pwd,'Data',app.t_folder.Value);
                    app.recName = strcat(app.dp_Gesture.Value,'_rec',num2str(app.recCounter));
                    fullRecPath = fullfile(app.recPath,app.recName);
                    
                    % Start acquisition
                    app.DAQDevice.startRec(fullRecPath);
                    app.isRecTrainData = 1;
                    
                    % reset emg rms buffer
                    app.EMGRms = [];
                    
                    % Update GUI 
                    app.b_cues.Text = 'Recording... | Stop';
                    app.b_cues.BackgroundColor = [1.00,0.41,0.16];
                    app.isPlotEMG.Enable = 1;
                else        
                    % Stop recording and save the data
                    app.RecInfo.EMGRMS = app.EMGRms;
                    app.DAQDevice.stopRec(app.RecInfo);
                    fprintf(strcat('Data saved on: ', app.recName,'.mat \n'));
                    
                    % Flags and counters
                    app.recCounter = app.recCounter + 1;
                    app.isRecTrainData = 0;
                    app.CursorSample = 1;
                    % Update gui
                    app.b_cues.Text = 'Record trial';
                    app.b_cues.BackgroundColor = [0.96,0.96,0.96];
                    app.isPlotEMG.Enable = 0;
                end
            end
        end

        % Button pushed function: b_recFolder
        function b_recFolderButtonPushed(app, event)
            % Get file
            app.recPath = uigetdir(fullfile(pwd,'Data'));
            [~,dir] = fileparts( app.recPath);
            % Update GUI
            app.t_folder.Value = dir;
            app.b_Reg.Enable = 'on';
            app.b_Train.Enable = 'on';
        end

        % Button pushed function: b_Reg
        function b_RegButtonPushed(app, event)
            % Get file
            if isempty(app.recPath)
                app.recPath = fullfile(pwd,'Data',app.t_folder.Value);
            end
            [file,path] = uigetfile(app.recPath);
            load(fullfile(path,file));
            fprintf(strcat('\n',file, ' regressor loaded!\n'));
            app.RegCoef = RegCoef;
            
            % Flags
            for i = 1: app.nDofs
                Order = 4;
                Fc = 1.5; 
                Fs = 0.04;
                app.softCtrl{i} = Filter('LowPass', Order, Fc, Fs);
            end
            app.isTrained = 1; 
            
            % Update GUI
            app.t_reg.Value = file;
            % Init robot model
            if isempty(app.myrobot)
                app.myrobot = MyRobot();
            end
        end

        % Value changed function: dp_Gesture
        function dp_GestureValueChanged(app, event)
            if strcmp(app.dp_Gesture.Value,'SupPron') ||  strcmp(app.dp_Gesture.Value,'RadUl')
                app.GestureLabel = app.LabelsRec{1};
                
            end
            if strcmp(app.dp_Gesture.Value,'OpnClose') ||  strcmp(app.dp_Gesture.Value,'FlexExt')
                app.GestureLabel = app.LabelsRec{2};
            end
            
            app.RecInfo.Labels = app.GestureLabel';
            
            % Reset the counter of cue recording
            app.recCounter = 1;
        end

        % Close request function: UIFigure
        function UIFigureCloseRequest(app, event)
            % Stop timers
            stop(app.ExeTimer);

            if exist('app.DAQDevice')
                if app.DAQDevice.isAcquiring
                    % Stop acquisition
                    app.DAQDevice.stopAcq();
                end
            end
            delete(app)
        end

        % Callback function
        function dp_HandPortValueChanged(app, event)
            % NO MODIFY BY STUDENT ! ---------------------------
            % Scan COM ports on the system
            if ismac
                [stat,result] = system('ls /dev/cu.*');
                start_indx = strfind(result,'/dev/');
                    for i = 1:size(start_indx,2)
                        if i==size(start_indx,2)
                            DevAvailable{i}=result(start_indx(i):end);
                            DevAvailable{i}=DevAvailable{i}(find(~isspace(DevAvailable{i})));
                        else
                            index_end=strfind(result( start_indx(i):end ),'/');
                            DevAvailable{i}=result(start_indx(i):start_indx(i)+index_end(3));
                            option_end=strfind(DevAvailable{i},'	');
                            if ~isempty(option_end)
                                DevAvailable{i}=DevAvailable{i}(1:option_end-1);
                            else 
                                option_end=regexp(DevAvailable{i}, '[\n]');
                                DevAvailable{i}=DevAvailable{i}(1:option_end-1);
                                DevAvailable{i}=DevAvailable{i}(find(~isspace(DevAvailable{i})));
                            end
                        end
                    end
            end
            if ispc
                DevAvailable = serialportlist;
            end
            
            app.dp_HandPort.Items = DevAvailable;
            
        end

        % Value changed function: cb_isHand
        function cb_isHandValueChanged(app, event)
            % If connect
            if app.cb_isHand.Value
                try
                    instrreset
                    % Create DAQ object
                    app.HandCom = serialport(app.dp_HandPort.Value, 115200);
                    
                    % Activate Connection flag
                    app.isHandConnected = 1;
                    pause(0.5);
                catch e
                    disp(e)
                    app.cb_isHand.Value = 0;
                    fprintf('Error!')
                end
            % If Disconnect
            else
                % Desactivate connection flag
                app.isHandConnected = 0;
                
                % Stop communication and delete 
                app.HandCom.delete();
                delete(app.HandCom);
            end
        end

        % Button pushed function: b_Train
        function b_TrainButtonPushed(app, event)
            
            ToScan  = fullfile(app.recPath,'/*rec*.mat');
            files   = dir(ToScan); 
            
            % Arrays to concatenate the data
            rawEMG      =[];          % Array for the raw EMG
            feats_aux   =[];       % Array for the rms EMG
            labels_aux  =[];      % Array for the training labels
            
            
            if isempty(files)
                fprintf('Get some data first! \n')
            else
                for nfile = 1: length(files)
                    % Load the file
                    FileToLoad  = fullfile(app.recPath, files(nfile).name);
                    Data = load(FileToLoad);
                    
                    fprintf(strcat('Loading file : ''', files(nfile).name, '''\n'));
                    
                    % Joint data
                    feats_aux   = [feats_aux;  Data.RecInfo.EMGRMS];
                    labels_aux  = [labels_aux; Data.RecInfo.Labels];
                    rawEMG      = [rawEMG;     Data.RecInfo.EMGraw];
                end
                
                %% 3.D - Train regressor
                labels  = labels_aux; % [DOFs x length recording] - [2 x Length recording]
                feats   = feats_aux;   % [channels x length recording] 
                app.feat2test = feats;
                RegCoef = T1B_TrainReg(feats, labels);
                app.RegCoef = RegCoef;
                save(fullfile(app.recPath,'CoefReg.mat'), 'RegCoef');
                
                csvwrite(fullfile(app.recPath,'CoefReg.csv'), RegCoef(:,1));
                
                
                fprintf('Regressor trained and saved! \n')
                
                % Update GUI
                app.b_Test.Enable = 'on';
            end
            
            % Flags
            for i = 1: app.nDofs
                Order = 4;
                Fc = 1; 
                Fs = 0.04;
                app.softCtrl{i} = Filter('LowPass', Order, Fc, Fs);
            end
            app.isTrained = 1; 
            
        end

        % Button pushed function: b_Test
        function b_TestButtonPushed(app, event)
            lengthRecording = length(app.feat2test);           
 
            % test the data by each sample            
            for sample = 1:lengthRecording
                trEst(sample,:) = T1C_TestReg([1, app.feat2test(sample,:)], app.RegCoef);
            end
            
            % Plot the prediction of the 2 DOF
            % Positive side represents Radio/Extension prediciton
            % Negative side represents Ulnar/Flexion
            f = figure,
            plot(trEst)
            legend('Prediction for DOF1','Prediction for DOF2');
            set(gca,'FontSize',20);
            xlabel('Sample [ud]');
            ylabel('Prediction [%]');
            
            f.Position = [127 790 1178 389];
        end

        % Button pushed function: b_Create
        function b_CreateButtonPushed(app, event)
            % Initialises the robot model
            app.myrobot = MyRobot();
        end

        % Value changed function: dp_HandPort
        function dp_HandPortValueChanged2(app, event)
            % NO MODIFY BY STUDENT ! ---------------------------
            % Scan COM ports on the system
            if ismac
                [stat,result] = system('ls /dev/cu.*');
                start_indx = strfind(result,'/dev/');
                    for i = 1:size(start_indx,2)
                        if i==size(start_indx,2)
                            DevAvailable{i}=result(start_indx(i):end);
                            DevAvailable{i}=DevAvailable{i}(find(~isspace(DevAvailable{i})));
                        else
                            index_end=strfind(result( start_indx(i):end ),'/');
                            DevAvailable{i}=result(start_indx(i):start_indx(i)+index_end(3));
                            option_end=strfind(DevAvailable{i},'	');
                            if ~isempty(option_end)
                                DevAvailable{i}=DevAvailable{i}(1:option_end-1);
                            else 
                                option_end=regexp(DevAvailable{i}, '[\n]');
                                DevAvailable{i}=DevAvailable{i}(1:option_end-1);
                                DevAvailable{i}=DevAvailable{i}(find(~isspace(DevAvailable{i})));
                            end
                        end
                    end
            end
            if ispc
                DevAvailable = serialportlist;
            end
            app.dp_HandPort.Items = DevAvailable;
        end

        % Drop down opening function: dp_HandPort
        function dp_HandPortDropDownOpening(app, event)
            % NO MODIFY BY STUDENT ! ---------------------------
            % Scan COM ports on the system
            %{
            if ismac
                [stat,result] = system('ls /dev/cu.*');
                start_indx = strfind(result,'/dev/');
                    for i = 1:size(start_indx,2)
                        if i==size(start_indx,2)
                            DevAvailable{i}=result(start_indx(i):end);
                            DevAvailable{i}=DevAvailable{i}(find(~isspace(DevAvailable{i})));
                        else
                            index_end=strfind(result( start_indx(i):end ),'/');
                            DevAvailable{i}=result(start_indx(i):start_indx(i)+index_end(3));
                            option_end=strfind(DevAvailable{i},'	');
                            if ~isempty(option_end)
                                DevAvailable{i}=DevAvailable{i}(1:option_end-1);
                            else 
                                option_end=regexp(DevAvailable{i}, '[\n]');
                                DevAvailable{i}=DevAvailable{i}(1:option_end-1);
                                DevAvailable{i}=DevAvailable{i}(find(~isspace(DevAvailable{i})));
                            end
                        end
                    end
            end
            if ispc
            %}
                DevAvailable = serialportlist;
            %end
            app.dp_HandPort.Items = DevAvailable;
        end

        % Callback function: FilterFreq, FilterFreq
        function FilterFreqValueChanged(app, event)
            for i = 1: app.nDofs
                Order = 4;
                Fc =  app.FilterFreq.Value;
                Fs = 0.04;
                app.softCtrl{i} = Filter('LowPass', Order, Fc, Fs);
            end
        end

        % Value changed function: isPlotEMG
        function isPlotEMGValueChanged(app, event)
            if app.isPlotEMG.Value
                % EMG plotting
                app.PlotEMG.YLim=[0,(app.DAQDevice.nCh+1)*app.PlotOffset+app.PlotOffset];
                app.PlotEMG.XLim=[0,1];
                ticks=linspace(0,1,5);
                app.PlotEMG.XTick=ticks;
                
                cmap = parula(app.DAQDevice.nCh); 
                cmap = flipud(cmap);
                % EMG primitive lines
                for nchan = 1:app.DAQDevice.nCh
                    app.lemg{nchan} = line(app.PlotEMG,nan,nan,'color',cmap(nchan,:),'linewidth',1);
                    app.PlotOffsets(nchan,:) = ones((app.DAQDevice.sampleRate*app.TimeWindowToPlot)-1,1) .* app.PlotOffset .* nchan;
                end
                % x axis time 
                app.xAxisTime=[1:1:app.DAQDevice.sampleRate*app.TimeWindowToPlot]/app.DAQDevice.sampleRate;
                
            else
                while(length(app.PlotEMG.Children)>0); delete(app.PlotEMG.Children(1)); end
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.AutoResizeChildren = 'off';
            app.UIFigure.Position = [100 100 1217 570];
            app.UIFigure.Name = 'MATLAB App';
            app.UIFigure.Resize = 'off';
            app.UIFigure.CloseRequestFcn = createCallbackFcn(app, @UIFigureCloseRequest, true);

            % Create DAQPanel
            app.DAQPanel = uipanel(app.UIFigure);
            app.DAQPanel.AutoResizeChildren = 'off';
            app.DAQPanel.Title = 'DAQ';
            app.DAQPanel.FontWeight = 'bold';
            app.DAQPanel.Position = [15 479 377 80];

            % Create PortDropDownLabel
            app.PortDropDownLabel = uilabel(app.DAQPanel);
            app.PortDropDownLabel.HorizontalAlignment = 'right';
            app.PortDropDownLabel.FontWeight = 'bold';
            app.PortDropDownLabel.Position = [1 29 30 22];
            app.PortDropDownLabel.Text = 'Port';

            % Create dp_Port
            app.dp_Port = uidropdown(app.DAQPanel);
            app.dp_Port.DropDownOpeningFcn = createCallbackFcn(app, @dp_PortValueChanged, true);
            app.dp_Port.ValueChangedFcn = createCallbackFcn(app, @dp_PortValueChanged, true);
            app.dp_Port.FontWeight = 'bold';
            app.dp_Port.Position = [37 29 81 22];

            % Create cb_isConnected
            app.cb_isConnected = uicheckbox(app.DAQPanel);
            app.cb_isConnected.ValueChangedFcn = createCallbackFcn(app, @cb_isConnectedValueChanged, true);
            app.cb_isConnected.Text = 'Connect';
            app.cb_isConnected.FontWeight = 'bold';
            app.cb_isConnected.Position = [191 29 70 22];

            % Create b_Acq
            app.b_Acq = uibutton(app.DAQPanel, 'push');
            app.b_Acq.ButtonPushedFcn = createCallbackFcn(app, @b_AcqPushed, true);
            app.b_Acq.FontWeight = 'bold';
            app.b_Acq.Enable = 'off';
            app.b_Acq.Position = [261 29 111 23];
            app.b_Acq.Text = 'Start Acquisition';

            % Create Dev
            app.Dev = uidropdown(app.DAQPanel);
            app.Dev.Items = {'ArmIO', 'MyoBand'};
            app.Dev.FontWeight = 'bold';
            app.Dev.Position = [120 29 68 22];
            app.Dev.Value = 'ArmIO';

            % Create isPlotEMG
            app.isPlotEMG = uicheckbox(app.DAQPanel);
            app.isPlotEMG.ValueChangedFcn = createCallbackFcn(app, @isPlotEMGValueChanged, true);
            app.isPlotEMG.Enable = 'off';
            app.isPlotEMG.Text = 'Enable time series visualisation';
            app.isPlotEMG.FontWeight = 'bold';
            app.isPlotEMG.Position = [4 5 203 22];

            % Create TrainingPanel
            app.TrainingPanel = uipanel(app.UIFigure);
            app.TrainingPanel.AutoResizeChildren = 'off';
            app.TrainingPanel.Title = 'Training';
            app.TrainingPanel.FontWeight = 'bold';
            app.TrainingPanel.Position = [399 479 210 80];

            % Create b_recFolder
            app.b_recFolder = uibutton(app.TrainingPanel, 'push');
            app.b_recFolder.ButtonPushedFcn = createCallbackFcn(app, @b_recFolderButtonPushed, true);
            app.b_recFolder.FontWeight = 'bold';
            app.b_recFolder.Position = [6 30 78 23];
            app.b_recFolder.Text = 'Rec Folder';

            % Create t_folder
            app.t_folder = uieditfield(app.TrainingPanel, 'text');
            app.t_folder.Editable = 'off';
            app.t_folder.FontWeight = 'bold';
            app.t_folder.Position = [89 31 117 22];

            % Create GestureDropDownLabel
            app.GestureDropDownLabel = uilabel(app.TrainingPanel);
            app.GestureDropDownLabel.HorizontalAlignment = 'right';
            app.GestureDropDownLabel.FontWeight = 'bold';
            app.GestureDropDownLabel.Position = [6 5 51 22];
            app.GestureDropDownLabel.Text = 'Gesture';

            % Create dp_Gesture
            app.dp_Gesture = uidropdown(app.TrainingPanel);
            app.dp_Gesture.Items = {'FlexExt', 'RadUl', 'SupPron', 'OpnClose'};
            app.dp_Gesture.ValueChangedFcn = createCallbackFcn(app, @dp_GestureValueChanged, true);
            app.dp_Gesture.Enable = 'off';
            app.dp_Gesture.FontWeight = 'bold';
            app.dp_Gesture.Position = [63 5 86 22];
            app.dp_Gesture.Value = 'SupPron';

            % Create b_cues
            app.b_cues = uibutton(app.TrainingPanel, 'push');
            app.b_cues.ButtonPushedFcn = createCallbackFcn(app, @b_cuesButtonPushed, true);
            app.b_cues.FontWeight = 'bold';
            app.b_cues.FontColor = [0.149 0.149 0.149];
            app.b_cues.Enable = 'off';
            app.b_cues.Position = [156 5 50 23];
            app.b_cues.Text = 'Record';

            % Create RegressorPanel
            app.RegressorPanel = uipanel(app.UIFigure);
            app.RegressorPanel.AutoResizeChildren = 'off';
            app.RegressorPanel.Title = 'Regressor';
            app.RegressorPanel.FontWeight = 'bold';
            app.RegressorPanel.Position = [616 463 187 96];

            % Create b_Reg
            app.b_Reg = uibutton(app.RegressorPanel, 'push');
            app.b_Reg.ButtonPushedFcn = createCallbackFcn(app, @b_RegButtonPushed, true);
            app.b_Reg.FontWeight = 'bold';
            app.b_Reg.Position = [9 8 74 23];
            app.b_Reg.Text = 'Load Reg.';

            % Create t_reg
            app.t_reg = uieditfield(app.RegressorPanel, 'text');
            app.t_reg.Editable = 'off';
            app.t_reg.FontWeight = 'bold';
            app.t_reg.Position = [86 8 97 22];

            % Create b_Train
            app.b_Train = uibutton(app.RegressorPanel, 'push');
            app.b_Train.ButtonPushedFcn = createCallbackFcn(app, @b_TrainButtonPushed, true);
            app.b_Train.FontWeight = 'bold';
            app.b_Train.Enable = 'off';
            app.b_Train.Position = [9 43 75 23];
            app.b_Train.Text = 'Train';

            % Create b_Test
            app.b_Test = uibutton(app.RegressorPanel, 'push');
            app.b_Test.ButtonPushedFcn = createCallbackFcn(app, @b_TestButtonPushed, true);
            app.b_Test.FontWeight = 'bold';
            app.b_Test.Enable = 'off';
            app.b_Test.Position = [86 43 97 23];
            app.b_Test.Text = 'Test';

            % Create HandPanel
            app.HandPanel = uipanel(app.UIFigure);
            app.HandPanel.AutoResizeChildren = 'off';
            app.HandPanel.Title = 'Hand';
            app.HandPanel.FontWeight = 'bold';
            app.HandPanel.Position = [815 463 393 96];

            % Create cb_isHand
            app.cb_isHand = uicheckbox(app.HandPanel);
            app.cb_isHand.ValueChangedFcn = createCallbackFcn(app, @cb_isHandValueChanged, true);
            app.cb_isHand.Text = 'Connect';
            app.cb_isHand.FontWeight = 'bold';
            app.cb_isHand.Position = [7 16 70 22];

            % Create PortDropDownLabel_2
            app.PortDropDownLabel_2 = uilabel(app.HandPanel);
            app.PortDropDownLabel_2.HorizontalAlignment = 'right';
            app.PortDropDownLabel_2.FontWeight = 'bold';
            app.PortDropDownLabel_2.Position = [7 42 30 22];
            app.PortDropDownLabel_2.Text = 'Port';

            % Create dp_HandPort
            app.dp_HandPort = uidropdown(app.HandPanel);
            app.dp_HandPort.DropDownOpeningFcn = createCallbackFcn(app, @dp_HandPortDropDownOpening, true);
            app.dp_HandPort.ValueChangedFcn = createCallbackFcn(app, @dp_HandPortValueChanged2, true);
            app.dp_HandPort.FontWeight = 'bold';
            app.dp_HandPort.Position = [43 42 81 22];

            % Create SmoothingFIlterFreqHzLabel
            app.SmoothingFIlterFreqHzLabel = uilabel(app.HandPanel);
            app.SmoothingFIlterFreqHzLabel.HorizontalAlignment = 'center';
            app.SmoothingFIlterFreqHzLabel.FontWeight = 'bold';
            app.SmoothingFIlterFreqHzLabel.Position = [170 24 91 30];
            app.SmoothingFIlterFreqHzLabel.Text = {'Smoothing'; 'FIlter Freq [Hz]'};

            % Create FilterFreq
            app.FilterFreq = uiknob(app.HandPanel, 'continuous');
            app.FilterFreq.Limits = [0 3];
            app.FilterFreq.ValueChangedFcn = createCallbackFcn(app, @FilterFreqValueChanged, true);
            app.FilterFreq.ValueChangingFcn = createCallbackFcn(app, @FilterFreqValueChanged, true);
            app.FilterFreq.FontWeight = 'bold';
            app.FilterFreq.Position = [296 20 39 39];
            app.FilterFreq.Value = 1.5;

            % Create ModelBasedPanel
            app.ModelBasedPanel = uipanel(app.UIFigure);
            app.ModelBasedPanel.AutoResizeChildren = 'off';
            app.ModelBasedPanel.Title = 'Model Based';
            app.ModelBasedPanel.Position = [1057 54 153 392];

            % Create cb_EnableRob
            app.cb_EnableRob = uicheckbox(app.ModelBasedPanel);
            app.cb_EnableRob.Text = 'Enable';
            app.cb_EnableRob.FontWeight = 'bold';
            app.cb_EnableRob.Position = [90 329 61 22];

            % Create b_Create
            app.b_Create = uibutton(app.ModelBasedPanel, 'push');
            app.b_Create.ButtonPushedFcn = createCallbackFcn(app, @b_CreateButtonPushed, true);
            app.b_Create.FontWeight = 'bold';
            app.b_Create.Position = [7 329 74 23];
            app.b_Create.Text = 'Create';

            % Create GainKnobLabel
            app.GainKnobLabel = uilabel(app.ModelBasedPanel);
            app.GainKnobLabel.HorizontalAlignment = 'center';
            app.GainKnobLabel.FontWeight = 'bold';
            app.GainKnobLabel.Position = [61 152 32 22];
            app.GainKnobLabel.Text = 'Gain';

            % Create Gain
            app.Gain = uiknob(app.ModelBasedPanel, 'continuous');
            app.Gain.FontWeight = 'bold';
            app.Gain.Position = [58 188 39 39];
            app.Gain.Value = 20;

            % Create ConstantExternalTorqueKnobLabel
            app.ConstantExternalTorqueKnobLabel = uilabel(app.ModelBasedPanel);
            app.ConstantExternalTorqueKnobLabel.HorizontalAlignment = 'center';
            app.ConstantExternalTorqueKnobLabel.FontWeight = 'bold';
            app.ConstantExternalTorqueKnobLabel.Position = [2 24 151 22];
            app.ConstantExternalTorqueKnobLabel.Text = 'Constant External Torque';

            % Create ExtTorque
            app.ExtTorque = uiknob(app.ModelBasedPanel, 'continuous');
            app.ExtTorque.FontWeight = 'bold';
            app.ExtTorque.Position = [58 79 39 39];

            % Create ExtensorChLabel
            app.ExtensorChLabel = uilabel(app.ModelBasedPanel);
            app.ExtensorChLabel.HorizontalAlignment = 'right';
            app.ExtensorChLabel.FontWeight = 'bold';
            app.ExtensorChLabel.Position = [8 293 76 22];
            app.ExtensorChLabel.Text = 'Extensor Ch';

            % Create ed_ExtensorCh
            app.ed_ExtensorCh = uieditfield(app.ModelBasedPanel, 'numeric');
            app.ed_ExtensorCh.FontWeight = 'bold';
            app.ed_ExtensorCh.Position = [99 293 42 22];
            app.ed_ExtensorCh.Value = 4;

            % Create FlexorChLabel
            app.FlexorChLabel = uilabel(app.ModelBasedPanel);
            app.FlexorChLabel.HorizontalAlignment = 'right';
            app.FlexorChLabel.FontWeight = 'bold';
            app.FlexorChLabel.Position = [24 263 60 22];
            app.FlexorChLabel.Text = 'Flexor Ch';

            % Create ed_FlexorCh
            app.ed_FlexorCh = uieditfield(app.ModelBasedPanel, 'numeric');
            app.ed_FlexorCh.FontWeight = 'bold';
            app.ed_FlexorCh.Position = [99 263 42 22];
            app.ed_FlexorCh.Value = 9;

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            zlabel(app.UIAxes, 'Z')
            app.UIAxes.Toolbar.Visible = 'off';
            app.UIAxes.PlotBoxAspectRatio = [1 1.02564102564103 1];
            app.UIAxes.XLim = [-100 100];
            app.UIAxes.YLim = [-100 100];
            app.UIAxes.BoxStyle = 'full';
            app.UIAxes.XGrid = 'on';
            app.UIAxes.YGrid = 'on';
            app.UIAxes.Box = 'on';
            app.UIAxes.Interruptible = 'off';
            app.UIAxes.HitTest = 'off';
            app.UIAxes.PickableParts = 'none';
            app.UIAxes.Position = [616 0 442 480];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = MyoControl

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end