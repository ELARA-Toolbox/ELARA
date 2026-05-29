classdef MBSimEnergies
    % Class to store the energy evolution of finished simulations
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        T (:,1) double      % Kinetic energy
        U (:,1) double      % Potential energy (due to gravity)
        V (:,1) double      % Strain energy
        H (:,1) double      % Total energy (T+U+V)
    end
end
