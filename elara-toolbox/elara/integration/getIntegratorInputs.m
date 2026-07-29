function u = getIntegratorInputs(MBSys, simPars, tout)
    %% Prepare system inputs at integration time steps for variational integrators
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments
        MBSys       (1,1) elara.abstract.System
        simPars     (1,1) elara.SimulationParameters
        tout        (:,1) double
    end

    nSteps = length(tout) - 1;

    % Time-varying system inputs
    if ~isempty(simPars.uSampleValues) && ~isempty(simPars.uSampleTimes)
        % Check dimensions
        assert( size(simPars.uSampleValues,2) == size(simPars.uSampleTimes,1), ...
            ['Nr. of sample times and sample values for time-varying inputs not equal', ...
            ' (Nr. of columns of uSampleValues does not match nr. of rows of uSampleTimes)']);

        % Interpolate input values
        u = interp1(simPars.uSampleTimes, simPars.uSampleValues.', tout, 'linear', 0).';
    else
        u = zeros(MBSys.nInputs, nSteps+1);
    end

    % Constant system inputs
    if ~isempty(simPars.uConst)
        u = u + repmat(simPars.uConst, [1, nSteps+1]);
    end
end