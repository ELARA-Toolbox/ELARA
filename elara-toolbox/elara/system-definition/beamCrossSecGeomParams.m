classdef beamCrossSecGeomParams
    % Class for beam cross-section geometry properties
    % (only properties of the cross-section geometry)
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich


    properties
        H      (1,1) double   % Height
        W      (1,1) double   % Width
        A      (1,1) double   % Cross-section area
        I_x    (1,1) double   % Second moments of inertia (about x axis of the body-fixed coordinate systems)
        I_y    (1,1) double   % Second moments of inertia (about y axis of the body-fixed coordinate systems)
        J_P    (1,1) double   % Polar moment of inertia
    end
end
