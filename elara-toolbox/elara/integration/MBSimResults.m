classdef MBSimResults
    % Class to store all simulation results of a beam simulation
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    properties
        %% Simulation Results Data
        eta   (6,:,:)   double  % se(3)/ R6 node velocity vectors            (6,   nLinks, nSteps)
        g     (4,4,:,:) double  % link/node configuration matrices           (4,4, nLinks, nSteps)
        q     (:,:)     double  % Generalized coordinates                    (nDoF, nSteps)
        q_dot (:,:)     double  % Generalized velocities                     (6,   nJoints, nSteps)
        tout  (:,1)     double  % time values                                (nSteps, 1)

        %% Solver Data

        % Nr of iterations of the implicit solver (1, nSteps)
        solverIterations (1,:) double

        % Residual error of the DEL equations after convergence (1, nSteps)
        solverError     (1,:) double

        % Exit flag of the implicit solver (Note: Meaning can depend on the
        % used solver) (1, nSteps)
        solverExitFlag  (1,:) double

        %% Meta Data of the Overall Simulation
        metaDataSim     (1,1) MBSimMetadata

        %% System Energies corresponding to the Simulation Results
        energies        (1,1) MBSimEnergies
        
    end

    methods
        function obj = MBSimResults(obj)
            % Create instance of this class
        end

        function obj = getSimMetaData(obj)
            % Compute the meta data values for the entire simulation from
            % the meta data of the individual steps and simulation results
            % data
            %{

            obj.metaDataSim.ImplicitIterations.min  = min( obj.solverIterations(:));
            obj.metaDataSim.ImplicitIterations.max  = max( obj.solverIterations(:));
            obj.metaDataSim.ImplicitIterations.mean = mean(obj.solverIterations(:), 'omitnan');

            obj.metaDataSim.ImplicitError.min       = min(  abs( obj.solverError(:) ));
            obj.metaDataSim.ImplicitError.max       = max(  abs( obj.solverError(:) ));
            obj.metaDataSim.ImplicitError.mean      = mean( abs( obj.solverError(:) ), 'omitnan');

            obj.metaDataSim.TotalIterations        = sum( obj.solverIterations(:), 'omitmissing');
            %}
        end
    end
end