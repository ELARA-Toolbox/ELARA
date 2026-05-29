classdef beamMaterialParams
    % Class for beam material properties 
    % (only properties of the material)
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich


    properties
        rho     (1,1) double    % Density (kg/m^3)
        E       (1,1) double    % Young's modulus (N/m^2)
        nu      (1,1) double    % Poisson's number
        G       (1,1) double    % Shear Modulus
    end
end
