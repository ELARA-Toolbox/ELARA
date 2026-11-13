classdef (Abstract) TendonActuation
    %% Class to define the actuation properties of tendon-actuated flexible links
    % used to model tendon-actuated continuum manipulators
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        % Local Transformations from Backbone to Tendons
        gBackboneTendon (4,4, :, :) % 4 x 4 x nTendons x nDisks

        % Local arc-length positions of the tendon-routing disks
        sDisks           (:,1) double {mustBeNonnegative}

        % Disk at which each tendon terminates
        terminationDisks (:,1) % nTendons x 1
    end
     properties (Abstract, Constant)
        % Whether the link is rigid or flexible
        tendonActuationType (1,:) string {mustBeMember(tendonActuationType,["continuous", "discrete"])}
    end
end
