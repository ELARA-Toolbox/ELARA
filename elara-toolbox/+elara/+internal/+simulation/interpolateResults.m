function [gQuery, qQuery] = interpolateResults(system, simRes, tQuery)
    %% Interpolate simulation results in time at fixed time grid
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments
        % System definition
        system  (1,1) elara.abstract.System

        % SimResults object containing the data to interpolate
        simRes  (1,1) elara.SimulationResults

        % Vector with time query points
        tQuery  (:,1) double
    end

    % Interpolate each generalized coordinate over time, including systems
    % with only one degree of freedom.
    qQuery = interp1(simRes.tout, simRes.q.', tQuery, 'linear').';

    % Forward kinematics at interpolated values
    gQuery = zeros(4,4, system.nFrames, numel(tQuery));
    for iStep = 1:numel(tQuery)
        gQuery(:,:,:,iStep) =  system.computeFwdKin(qQuery(:,iStep));
    end
end
