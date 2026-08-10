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

        % Initial control and configuration
        u0      (:,1)
        q0      (:,1)

        % Initial and final (generalized) velocity
        qDot0   (:,1)
        qDotF   (:,1)    % Leave empty to disable final velocity constraint

        % Control limits
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


        %% Control parameterization

        % Parameterize the control trajectory with B-splines?
        useSplineInputs (1,1) logical = false;

        % Number of control points for the control spline
        nInputSplinePoints (1,1) double = 10;

        % Order of the B-spline (standard: cubic)
        inputSplineOrder (1,1) double = 3;


        %% Final time configuration

        % Desired TCP pose at the final time
        % If left empty, it will not be considered
        x_TCP_F (3,1)

        % Desired rotation (not implemented yet)
        %R_TCP_F (:,3) % Must be 3x3 rotation matrix, but can be left empty


        % Whether to add a constraint for the final time TCP position
        addTCPFinalTimeConstraint (1,1) logical = false;

        % Coordinates at the final time:
        % Will be enforced as constraints at tEnd
        % If left empty, will not be considered
        qF      (:,1)

        %% TCP trajectory tracking

        x_TCP_traj          (3,:) double;

        % Pre- and post-actuation durations during which the initial-guess
        % trajectory is held constant (required for trajectory tracking)
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
        % Number of time intervals in the OCP
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

        function MBSim = getSimulationObject(obj)
            %% Create a simulation object from the problem definition
            arguments
                obj (1,1) elara.ocp.Problem
            end

            MBSim = elara.Simulation;
            MBSim.links = obj.links;
            MBSim.system = obj.systemNum;
            MBSim.parameters = obj.simPars;
        end

        %% Solver methods
        function obj = initSolver(obj, opts)
            %% Initialize NLP solver for the OCP
            arguments
                obj     (1,1) elara.ocp.Problem

                % Use a CasADi function to evaluate the DEL equations at each time step
                opts.useCasadiStepFunctions (1,1) logical = false;

                % Draw debug plots (constraint Jacobian etc.)?
                opts.showDebugPlots (1,1) logical = false;
            end
            [obj.NLPSolver, obj.constrDef] = elara.internal.ocp.initializeSolver(obj, ...
                "showDebugPlots", opts.showDebugPlots, ...
                "useCasadiStepFunctions", opts.useCasadiStepFunctions);

        end
        function [x_sol, u_sol, u_sol_z, sol, stats] = solve(obj, xInit, uInit, opts)
            %% Solve OCP
            arguments (Input)
                obj         (1,1) elara.ocp.Problem

                % Initial guess configuration or state trajectory
                % Variational discretization: configurations q, (nDoF, nSteps+1)
                % ODE discretization: states x = [q; qDot], (2*nDoF, nSteps+1)
                xInit   (:,:) double

                % Initial guess control decision variables
                % Direct parameterization: values at the time nodes,
                % (nInputs, nSteps+1)
                % Spline parameterization: B-spline control points,
                % (nInputs, nSplinePoints)
                uInit   (:,:) double

                % Struct with the results of a previous solver run;
                % if given, used to initialize/warm-start the solver
                opts.solWarmStart    (1,1) struct = struct();
            end
            arguments (Output)
                % Solved configuration or state trajectory, analogous to
                % xInit
                x_sol

                % Solved control trajectory evaluated at the time nodes,
                % (nInputs, nSteps+1)
                u_sol

                % Solved control decision variables
                % Direct parameterization: time-node values
                % (nInputs, nSteps+1), with u_sol_z equal to u_sol
                % Spline parameterization: B-spline control points
                % (nInputs, nSplinePoints)
                u_sol_z

                % CasADi NLP solver solution object
                sol

                % CasADi NLP solver metadata/stats object
                stats
            end
            % Check if solver has been defined
            assert(~strcmp(obj.NLPSolver.name, 'null') && isfield(obj.constrDef, "lb_c"), ...
                "Cannot start solver: NLP solver not initialized. Call initSolver() first.");

            % Solve problem
            [x_sol, u_sol, u_sol_z, sol, stats] = elara.internal.ocp.solve(obj, xInit, uInit, ...
                "solWarmStart", opts.solWarmStart);
        end
        function obj = clearSolver(obj)
            %% Clear the NLP Solver variables
            % e.g., to free up memory
            obj.NLPSolver = casadi.Function;
            obj.constrDef = struct();
        end

        %% Other Methods
        function fh = plotConstraintResiduals(obj, x, u_z, opts)
            %% Plot OCP constraint residuals for a given trajectory
            arguments
                obj         (1,1) elara.ocp.Problem

                % Configuration or state trajectory
                % Variational discretization: configurations q, (nDoF, nSteps+1)
                % ODE discretization: states x = [q; qDot], (2*nDoF, nSteps+1)
                x       (:,:) double

                % Control decision variables
                % Direct parameterization: values at the time nodes, (nInputs, nSteps+1)
                % Spline parameterization: B-spline control points, (nInputs, nSplinePoints)
                u_z     (:,:) double

                opts.figureName (1,1) string = "Constraint Residuals";
            end
            fh = elara.ocp.plot.constraintResiduals(obj, x, u_z, ...
                "figureName", opts.figureName);
        end
        function [B, B_dt, B_ddt, tau] = getInputSplineBasisMatrix(obj, opts)
            % Compute the basis matrix of the control B-spline
            % (and its derivatives)
            arguments
                obj                     (1,1) elara.ocp.Problem

                % Normalized stage locations within each time interval
                % defined by OCP.tout.
                % For each element of the vector, an additional time
                % instance t_k_i is added to the overall time vector,
                % where t_k_i = t_k + c(i)*h.
                opts.stageValues        (:,1) double {mustBeNonnegative} = 0;
            end
            tMat = opts.stageValues*obj.h + obj.tout.';
            % Make sure highest time value is the end time
            tMat(tMat>obj.tout(end)) = obj.tout(end);
            tEval = tMat(:);
            [B, B_dt, B_ddt, tau] = elara.internal.ocp.computeBSplineBasis( ...
                obj.nInputSplinePoints, obj.inputSplineOrder, tEval, ...
                obj.tout(1), obj.tout(end));
        end
        function [J, cR, cF] = evaluateObjectiveComponents(obj, x, u_z)
            %% Evaluate the objective function for a given trajectory
            % computes both the overall cost and the individual components
            arguments
                obj (1,1) elara.ocp.Problem

                % Configuration or state trajectory
                % Variational discretization: configurations q, (nDoF, nSteps+1)
                % ODE discretization: states x = [q; qDot], (2*nDoF, nSteps+1)
                x       (:,:) double

                % Control decision variables
                % Direct parameterization: values at the time nodes, (nInputs, nSteps+1)
                % Spline parameterization: B-spline control points, (nInputs, nSplinePoints)
                u_z     (:,:) double
            end
            % Check if solver is initialized
            if isfield(obj.constrDef, "Fun_fRComp")
                % Assemble decision variable vector
                if obj.useSplineInputs
                    X = elara.internal.ocp.packSplineDecisionVariables(x, u_z);
                else
                    X = elara.internal.ocp.packNodeDecisionVariables(x, u_z);
                end

                % Total cost
                J = cellfun( @(x) full(x), ...
                    obj.constrDef.Fun_f.call( ...
                    {X, obj.x_TCP_F, obj.runningCostWeights, obj.finalCostWeights}) ...
                    );

                % Running cost components
                cR = cellfun( @(x) full(x), ...
                    obj.constrDef.Fun_fRComp.call( ...
                    {X, obj.x_TCP_F, obj.runningCostWeights}) ...
                    );

                % Final cost components
                cF = cellfun( @(x) full(x), ...
                    obj.constrDef.Fun_fFComp.call( ...
                    {X, obj.x_TCP_F, obj.finalCostWeights}) ...
                    );
            else
                warning("Objective components cannot be evaluated " + ...
                    "if the solver is not initialized. " + ...
                    "Initialize the solver first.");
            end
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
