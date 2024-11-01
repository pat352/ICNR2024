function DataOutRms = T1A_ExtractRms(DataInRaw)
    % Task 1. Implement RMS function of a given EMG time window.
    % To complete by the student

    % Implement RMS function of a given EMG time window.
    % DataInRaw Input format : [samples x number of Channels] - [160 x 12]
    % DataOutRms Output format : [1 x number of Channels] - [1 x 12]

    DataOutRms = sqrt(sum(DataInRaw.^2)/size(DataInRaw,1));
            
end