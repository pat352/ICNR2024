%% PROSTHESIS MYOCONTROL WORKSHOP 

% Content:
% 1. Plotting individual trials of recorded data
% 2. Plot the rms signals over the time
% 3. 
% 3.A - Concatenate all data
% 3.B - Extract features: RMS  --- To complete by the student ------------
% 3.C -  Plotting of the training data and labels
% 3.D - Train regressor  --- To complete by the student ------------------
% 3.E - Test the regressor output --- To complete by the student ------------------
% 3.F - Save the regressor
% 3.G - Test the regressor with new Data 

%% 1. Plot trial
%close all;
% Get the file
[nfile,path] = uigetfile();
file         = load(fullfile(path,nfile));

% Input data to test with the regressor
RecInfo      = file.RecInfo;       % Rms data to test

figure(1)
nchan        = size(RecInfo.EMGRMS,2);                        % define the number of channels
% Plotting function
% plotCh(data, number of channels, Sampling frequency)
dataToPlot   = [RecInfo.EMGRMS';RecInfo.Labels'];           % to include in the plot the labels
plotCh(dataToPlot.*10, nchan+2, 25)                   % Plot data and labels, 25 Hz frequency

% Plot of the rms Data
figure(2)
plotCh(RecInfo.EMGraw', nchan, 1000)

%% 2. Plot over time
%close all;

FP       = figure(3);
FP.Color = [1,1,1];
rlim     = max(max(RecInfo.EMGRMS));
%theta    = deg2rad([(36):36:(360)]);
theta    = deg2rad([0:360/nchan:360]);
for sample = 1 : length(RecInfo.EMGRMS)

    FP = polarplot(theta(1:end-1),RecInfo.EMGRMS(sample,1:nchan),'o-','linewidth',2,'color',[0, 0.8, 0.8]);
    %FP = polarplot(theta,RecInfo.EMGRMS(sample,[2:10,1]),'o-','linewidth',2,'color',[0, 0.8, 0.8]);

    hold off
    FP.Parent.RLim = [0,rlim];

    pause(0.04)
    drawnow;
end

%% 3.A - Concatenate all data

% Concatenate data
path    = uigetdir();
% Scan all the recording files in the folder
ToScan  = fullfile(path,'/*rec*.mat');
files   = dir(ToScan); 

% Arrays to concatenate the data
rawEMG      =[];          % Array for the raw EMG
feats_aux   =[];       % Array for the rms EMG
labels_aux  =[];      % Array for the training labels
 
for nfile = 1: length(files)
    % Load the file
    FileToLoad  = fullfile(path,files(nfile).name);
    load(FileToLoad);
    
    fprintf(strcat('Loading file : ''', files(nfile).name, '''\n'));
    
    % Joint data
    feats_aux   = [feats_aux;  RecInfo.EMGRMS];
    labels_aux  = [labels_aux; RecInfo.Labels];
    rawEMG      = [rawEMG;     RecInfo.EMGraw];
end


%% 3.B - Extract features: RMS
Rmswindow   = 160; 
BufSize     = 40;
samplect    = 1;
nchan       = size(feats_aux,2);

% RMS data output
dOut        = zeros(round(length(rawEMG)/BufSize) , nchan);


for n = 1:round(length(rawEMG)/BufSize)-(Rmswindow/BufSize)

    if n == 1 
        DataI = rawEMG(1:Rmswindow,:);
    else
        DataI = rawEMG((BufSize * (n-1) + 1):(BufSize * (n-1)) + Rmswindow,:);
    end
    % FUNCTION TO IMPLEMENT BY THE STUDENT --------------------------------------
    % DataRmsEMG = T1A_ExtractRMS(DataInRawEMG)
    
    dOut(samplect,:) = T1A_ExtractRms(DataI);

    % TO COMPLETE BY THE STUDENT --------------------------------------
    samplect = samplect + 1;
    
end

plotCh(dOut'.*10, nchan, 25)                   % Plot data and labels, 25 Hz frequency

%% 3.C -  Plotting of the training data and labels

% Plot of the raw Data and labels
figure(2);
nchan = size(feats_aux,2);
plotCh([feats_aux';labels_aux'].*10, nchan+2, 25)


% Plot of the rms Data
figure(3)
plotCh(rawEMG', nchan, 1000)

%% 3.D - Train regressor  --- To complete by the student
labels  = labels_aux; % [DOFs x length recording] - [2 x Length recording]
feats   = feats_aux;   % [channels x length recording] 

 % FUNCTION TO IMPLEMENT BY THE STUDENT --------------------------------------
RegCoef = T1B_TrainReg(feats, labels);

%% 3.E Test the regressor output

lengthRecording = length(feats);           
 
% test the data by each sample            
for sample = 1:lengthRecording
    trEst(sample,:) = T1C_TestReg([1,feats(sample,:)], RegCoef);
end

% Plot the prediction of the 2 DOF
% Positive side represents Radio/Extension prediciton
% Negative side represents Ulnar/Flexion
plot(trEst)
legend('Prediction for Radio/Ulnar','Prediction for Extension/Flexion');
set(gca,'FontSize',20);
xlabel('Sample [ud]');
ylabel('Prediction [%]');
%% 3.F - Save the regressor
save(fullfile(path,'CoefReg.mat'), 'RegCoef');

fprintf('Regressor completely saved! \n')
%% 3.G - Test the regressor with new Data
% Get the file
[nfile,path] = uigetfile();
file         = load(fullfile(path,nfile));

% Input data to test with the regressor
inputData    = file.RecInfo.EMGRMS;       % Rms data to test
lengthRecording = length(inputData);  

% test the data by each sample            
for sample = 1:lengthRecording
    OutputPred(sample,:) = T1C_TestReg([1,inputData(sample,:)], RegCoef);
end

% Plot the prediction
plot(OutputPred)
% Positive side represents Radio/Extension prediciton
% Negative side represents Ulnar/Flexion
title(nfile(3:end-4))
legend('Prediction for Radio/Ulnar','Prediction for Extension/Flexion');
set(gca,'FontSize',20);
xlabel('Sample [ud]');
ylabel('Prediction [%]');