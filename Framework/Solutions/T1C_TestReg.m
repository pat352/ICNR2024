function Estimate = T1C_TestReg(feats, RegCoef)
    % Task 1C. Function to test the regressor
    % To complete by the student
    
    % Inputs:
    % feats : EMG RMS data               - [channels + 1 linear constant x length recording] 
    % RegCoef : regression coef. matrix  - [number of channels + 1 x DoFs]
    
    % Output:
    % trEst : Prediction output          - [DOF outputs x EMG rms samples to test]    
    
    % TO COMPLETE BY THE STUDENT ----------------------------------------------

    Estimate = feats * RegCoef;

    % --------------------------------------------------------------------
    
end