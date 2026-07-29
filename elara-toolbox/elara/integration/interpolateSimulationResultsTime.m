function [gQuery, qQuery] = interpolateSimulationResultsTime(MBSys, simRes, tQuery)
    %% Interpolate simulation results in time at fixed time grid
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments
        % System definition
        MBSys   (1,1) elara.abstract.System

        % SimResults object containing the data to interpolate
        simRes  (1,1) elara.SimulationResults

        % Vector with time query points
        tQuery  (:,1) double
    end

    % Get interpolated coordinate vector
    qQuery = interpn( ...
        1:size(simRes.q,1), simRes.tout, ...
        simRes.q, ...
        1:size(simRes.q,1), tQuery);

    % Forward kinematics at interpolated values
    gQuery = zeros(4,4, MBSys.nFrames, numel(tQuery));
    for iStep = 1:numel(tQuery)
        gQuery(:,:,:,iStep) =  MBSys.computeFwdKin(qQuery(:,iStep));
    end
end
