classdef Problem
    %% Class that fully defines an Optimal Control Problem (OCP)
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        Name    (1,1) string

        % End time
        tEnd      (1,1)

        % Sample time
        h       (1,1)

        % Initial values for inputs and coordinates
        u0      (:,1)
        q0      (:,1)

        % Initial and final (generalized) velocity
        qDot0   (:,1)
        qDotF   (:,1)    % Leave empty to disable final velocity constraint

        % Input limits
        uMin    (:,1)
        uMax    (:,1)

        % Coordinate limits
        qMin    (:,1)
        qMax    (:,1)

        % simPars object
        simPars (1,1) elara.SimulationParameters

        % Workspace definition
        workspace    (1,1) elara.Workspace

        % Struct with CasADi NLP options (given to the NLP object)
        nlpOptions         (1,1) struct

        %% System Definition
        % Important: There is currently no synchronization between the
        % links and numeric and symbolic system objects!
        % I.e., if a parameter in one of the system objects is changed,
        % the corresponding parameter in the other object must be manually
        % updated!

        % Link definitions
        links       (:,1) elara.abstract.Link

        % Numeric system representation
        systemNum   (1,1) elara.SystemNum

        % Symbolic system representation used for CasADi functions
        systemSym   (1,1) elara.SystemSym

        %% Cost function definition
        % Weight vectors define the contribution of each cost term.
        % Active vectors select which corresponding terms are included.

        % Running cost with elements:
        %  Norm u
        %  Norm u_dot
        %  Norm u_ddot
        %  Norm q_ddot
        %  TCP error (trajectory tracking)
        runningCostWeights     (5,1) double
        runningCostActive      (5,1) logical = true;

        % Final time cost with elements:
        %   Norm u
        %   Norm q
        %   TCP Error
        finalCostWeights     (3,1) double
        finalCostActive      (3,1) logical = true;

        % Order of the finite differences used for the derivatives of u and
        % q in the cost function
        finiteDifferenceOrder (1,1) double {mustBeMember(finiteDifferenceOrder, [2,4])} = 2;


        %% Input parameterization

        % Parameterize input trajectory with B-Splines?
        useSplineInputs (1,1) logical = false;

        % Nr. of control points for the input spline
        nInputSplinePoints (1,1) double = 10;

        % Order of the B-spline (standard: cubic)
        inputSplineOrder (1,1) double = 3;


        %% Final time configuration

        % Desired TCP pose at the final time
        % If left empty, it will not be considered
        R_TCP_F (:,3) % Must be 3x3 rotation matrix, but can be left empty
        x_TCP_F (3,1)

        % Whether to add a constraint for the final time TCP position
        addTCPFinalTimeConstraint (1,1) logical = false;

        % Coordinates at the final time:
        % Will be enforced as constraints at tEnd
        % If left empty, will not be considered
        qF      (:,1)

        %% TCP trajectory tracking

        x_TCP_traj          (3,:) double;

        % Time spans for pre and post actuation (i.e., duration, at which
        % the initial guess trajectory is held constant at the beginning
        % and at the end; required for trajectory tracking)
        tPreAct             (1,1) double = 0;
        tPostAct            (1,1) double = 0;

        % Waypoints for initial guess generation
        x_TCP_waypoints     (3,:) double
        x_TCP_timepoints    (1,:) double


        %% Discretization
        % Object defining the OCP discretization
        discretization (1,1) elara.abstract.OCPDiscretization = elara.ocp.DiscretizationVI;
    end
    properties(SetAccess=protected)
        %% "Internal" properties for solver definition

        % CasADi NLP solver object
        NLPSolver   (1,1) casadi.Function

        % Struct with constraint definition (mainly lb, ub)
        constrDef   (1,1) struct
    end

    properties (Dependent)
        % Nr. of time intervals in the OCP
        nSteps  (1,1)

        % Time vector of the OCP
        tout    (1,:)
    end

    %% Methods
    methods
        %% Constructor
        function obj = Problem(links)
            % Create an optimal control problem and optionally assemble its
            % numeric and symbolic systems from link definitions.
            arguments
                links (:,1) elara.abstract.Link = elara.RigidLink.empty;
            end

            if ~isempty(links)
                obj.links = links;
                obj.systemNum = elara.SystemNum(links);
                obj.systemSym = elara.SystemSym(links);
            end
        end

        %% Solver methods
        function obj = initSolver(obj, opts)
            %% Initialize NLP solver for the OCP
            arguments
                obj     (1,1) elara.ocp.Problem

                % Use a casadi function to evaluate the DEL in each time step
                opts.useCasadiStepFunctions (1,1) logical = false;

                % Draw debug plots (constraint Jacobian etc.)?
                opts.showDebugPlots (1,1) logical = false;
            end
            [obj.NLPSolver, obj.constrDef] = elara.internal.ocp.initializeSolver(obj, ...
                "showDebugPlots", opts.showDebugPlots, ...
                "useCasadiStepFunctions", opts.useCasadiStepFunctions);

        end
        function [x_sol, u_sol_z, sol, stats] = solve(obj, xInit, uInit, opts)
            %% Solve OCP
            arguments
                obj         (1,1) elara.ocp.Problem

                % Initial guess
                xInit       (:,:) double % can be configuration q or state x!
                uInit       (:,:) double

                % Struct with the results of a previous solver run;
                % if given, used to initialize/warm-start the solver
                opts.solWarmStart    (1,1) struct = struct();
            end
            % Check if solver has been defined
            assert(~strcmp(obj.NLPSolver.name, 'null') && isfield(obj.constrDef, "lb_c"), ...
                "Cannot start solver: NLP solver not initizalized. Call initSolver() first.");

            % Solve problem
            [x_sol, u_sol_z, sol, stats] = elara.internal.ocp.solve(obj, xInit, uInit, ...
                "solWarmStart", opts.solWarmStart);
        end
        function obj = clearSolver(obj)
            %% Clear the NLP Solver variables
            % e.g., to free up memory
            obj.NLPSolver = casadi.Function;
            obj.constrDef = struct();
        end
        
        %% Other Methods
        function fh = plotConstraintResiduals(obj, q, u, opts)
            %% Plot OCP Constraints residuals for a given trajectory
            arguments
                obj         (1,1) elara.ocp.Problem

                % Trajectory, for which the residuals should be evaluated
                q       (:,:) double % (nDoF, nSteps+1) or (2*nDoF, nSteps+1)
                u       (:,:) double % (nInputs, nSteps+1)

                opts.figureName (1,1) string = "Constraint Residuals";
            end
            fh = elara.ocp.plot.constraintResiduals(obj, q, u, ...
                "figureName", opts.figureName);
        end
        function [B, B_dt, B_ddt, tau] = getInputSplineBasisMatrix(obj, opts)
            % Compute the basis matrix of the input B-spline
            % (and its derivatives)
            arguments
                obj                     (1,1) elara.ocp.Problem

                % Vector of stage values / sub-samples of input values in
                % each standard time interval defined by OCP.tout.
                % For each element of the vector, an additional time
                % instance t_k_i is added to the overall time vector,
                % where t_k_i = t_k + c(i)*h.
                opts.stageValues        (:,1) double {mustBeNonnegative} = 0;
            end
            tMat = opts.stageValues*obj.h + obj.tout.';
            % Make sure highest time value is the end time
            tMat(tMat>obj.tout(end)) = obj.tout(end);
            tEval = tMat(:);
            [B, B_dt, B_ddt, tau] = computeBSplineBasisMatrix( ...
                obj.nInputSplinePoints, obj.inputSplineOrder, tEval, ...
                obj.tout(1), obj.tout(end));
        end
    end

    %% Get methods
    methods
        function nSteps = get.nSteps(obj)
            nSteps = round(obj.tEnd/obj.h);
        end
        function tout = get.tout(obj)
            tout = (0 : obj.h : obj.h*obj.nSteps).';
        end
    end
end
