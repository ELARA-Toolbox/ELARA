classdef TendonActuation
    %% Class to define the actuation properties of cable actuated flexible links
    % used to model cable-actuated continuum manipulators
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        % Local Transformations from Backbone to Tendons
        gBackboneTendon (4,4, :, :) % 4 x 4 x nTendons x nDisks

        % Local arc-length positions of the tendon-routing disks.
        % If empty, the beam-frame positions are used during assembly.
        sDisks           (:,1) double {mustBeNonnegative}

        % Disk at which each tendon terminates
        terminationDisks (:,1) % nTendons x 1
    end
     
end
