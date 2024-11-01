function RegCoef = T1B_TrainReg(feats, labels)
    % Task 2. Implement a linear regressor training function
    % To complete by the student
    
    % Inputs:
    % feats : EMG RMS data  - [ 1 + channels x length recording] 
    % labels: labels of the recording : [DOFs x length recording] - [2 x Length recording]

    % Output:
    % RegCoef : regressor coeficients - [ (number of channels + 1) x DOFs ]
    
    % Assisitve variables
    estimatedDOF = 2;                                 % Dof to estimate : 2
    nchannels = size(feats,2);                        % Number of channels of the device
    lengthRecording = length(labels);

    % Input:
    feats = [ones(lengthRecording,1) , feats];         % Rms features of the EMG
    % The extra column of ones we add to the features it to add the
    % constant of the linear regressor
    
    % Output:
    RegCoef = zeros(nchannels + 1,estimatedDOF);        % regressor coeficients - [ (number of channels + 1) x DOFs ]
    
    % TO COMPLETE BY THE STUDENT ----------------------------------------------

    for iDOF = 1:2
        % Train each DOF 
        RegCoef(:,iDOF) = regress( labels(:,iDOF) , feats )';
    end

    % --------------------------------------------------------------------  
    
    fprintf('Regressor trained!\n\nCoefficients:\n')
    disp(RegCoef)
    
    if size(RegCoef,1) ~= (nchannels + 1) || size(RegCoef,2) ~= estimatedDOF
        fprintf('Incorrect regressor dimensions!!\n')
    end
    
end