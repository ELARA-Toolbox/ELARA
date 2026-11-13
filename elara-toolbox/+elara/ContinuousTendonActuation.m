classdef ContinuousTendonActuation < elara.abstract.TendonActuation
    %% Class defining tendon actuation with continuously routed tendons
    % (e.g., inside channels)
    properties (Constant)
        tendonActuationType = "continuous";
    end
    properties (SetAccess=private)
        % Vector of lengths, at which the tendons terminate (nTendons,1)
        LTermination    (:,1) double {mustBeFinite,mustBePositive}

        % Cell array of function handles that define the tendon path
        % (tendon position relative to the backbone/centerline) over beam
        % length. Dimensions: (nTendons, 1)
        % The functions must have the form
        %           [x;y;z] = f(s)
        % where f : R -> R^3 returns the tendon path at s
        x_td_funs        (:,1) cell

        % First and second derivatives of the tendon path functions w.r.t. s
        x_td_ds_funs     (:,1) cell = {}
        x_td_dds_funs    (:,1) cell = {}
    end
    methods
        function obj = ContinuousTendonActuation(LTermination, L, nSegments, x_td_funs, x_td_ds_funs, x_td_dds_funs)
            arguments
                % Vector of lengths, at which the tendon terminate (nTendons,1)
                LTermination    (:,1) double {mustBeFinite,mustBePositive}

                % Overall beam length
                L               (1,1) double {mustBeFinite,mustBePositive}

                % Number of discrete segments of the beam
                nSegments       (1,1) double {mustBeFinite,mustBePositive}

                % Cell array of function handles that define the tendon path
                x_td_funs        (:,1) cell

                % First and second derivatives of the tendon path functions w.r.t. s
                x_td_ds_funs     (:,1) cell = {}
                x_td_dds_funs    (:,1) cell = {}
            end

            % Make sure argument sizes match
            assert(numel(LTermination) == numel(x_td_funs));

            obj.x_td_funs = x_td_funs;
            obj.x_td_ds_funs = x_td_ds_funs;
            obj.x_td_dds_funs = x_td_dds_funs;
            obj.LTermination = LTermination;

            % Compute disc arc-length locations (at the beam nodes)
            obj.sDisks = linspace(0,L,nSegments+1).';
            [obj.gBackboneTendon, obj.terminationDisks] = getNodeData(obj, obj.sDisks);
        end

        function [gBackboneTendon, terminationDisks] = getNodeData(obj, sDisks)
            arguments
                obj     (1,1) elara.ContinuousTendonActuation
                sDisks  (:,1) double {mustBeNonnegative,mustBeFinite}
            end

            if isempty(obj.x_td_ds_funs)
                s = sym("s");
                obj.x_td_ds_funs = cellfun( ...
                    @(x) matlabFunction( diff(x(s), s, 1), "Vars", s ), ...
                    obj.x_td_funs, ...
                    'UniformOutput', false);
            end
            if isempty(obj.x_td_dds_funs)
                s = sym("s");
                obj.x_td_dds_funs = cellfun( ...
                    @(x) matlabFunction( diff(x(s), s, 2), "Vars", s ), ...
                    obj.x_td_funs, ...
                    'UniformOutput', false);
            end

            % Make sure function array sizes match
            assert(length(obj.x_td_funs) == length(obj.x_td_ds_funs));
            assert(length(obj.x_td_funs) == length(obj.x_td_dds_funs));
            assert(length(obj.x_td_funs) == length(obj.LTermination));

            nDisks  = length(sDisks);
            nTendons = length(obj.x_td_funs);

            % Compute relative transformation / position of the tendon path
            % at the nodes
            gBackboneTendon  = repmat(eye(4), [1,1,nTendons,nDisks]);
            terminationDisks = zeros(nTendons, 1);

            for iTendon = 1:nTendons
                % Tendon positions
                xBackboneTendons = cell2mat( ...
                    arrayfun( obj.x_td_funs{iTendon}, sDisks , ...
                    'UniformOutput', false).' ...
                    );

                %%% Tangential tendon path rotation matrices (Frenet-Serret
                %%% frames)
                R = zeros(3, 3, nDisks);

                % Get function handles
                f_ds  = obj.x_td_ds_funs{iTendon};
                f_dds = obj.x_td_dds_funs{iTendon};

                % Compute basis vectors of the Frenet-Serret frame
                for iDisk = 1:nDisks
                    sDisk = sDisks(iDisk);
                    % Tangent vector t
                    t = f_ds(sDisk);% / norm(f_ds(lNodes(iN)));
                    t(3) = 1;
                    t = t / norm(t);

                    % Normal direction: component of the second derivative
                    % perpendicular to the unit tangent
                    a = f_dds(sDisk);
                    n = a - t * (t.' * a);

                    % At zero curvature, choose a perpendicular reference direction
                    if norm(n) <= 100 * eps * norm(a)
                        [~, iAxis] = min(abs(t));
                        e = zeros(3,1);
                        e(iAxis) = 1;
                        n = e - t * (t.' * e);
                    end

                    n = n / norm(n);

                    % Bi-normal vector b
                    b = cross(n, t);
                    b = b / norm(b);

                    % "Re-orthogonalize" the frame
                    n = cross(t, b);

                    % Store in rotation matrix; z-axis is aligned with
                    % tangent vector
                    R(:,:,iDisk) = [b, n, t];
                end

                % Tendon mount transformations
                gBackboneTendon(1:3,4, iTendon, :) = xBackboneTendons;
                gBackboneTendon(1:3,1:3, iTendon, :) = R;

                % Compute disk at which each tendon ends
                [~, terminationDisks(iTendon)] = min(abs(sDisks - obj.LTermination(iTendon)));

                % For short tendons (shorter than the segment length), use
                % the second disk to avoid zero-length tendons
                terminationDisks(iTendon) = max(terminationDisks(iTendon), 2);
            end

            % Validate computed transformations
            try
                elara.internal.validation.mustBeSE3Matrix(gBackboneTendon);
            catch cause
                exception = MException( ...
                    "elara:ContinuousTendonActuation:InvalidRoutingTransforms", ...
                    "Could not construct valid SE3 tendon routing transformations. " + ...
                    "Check the supplied tendon path functions and their derivatives.");

                exception = addCause(exception, cause);
                throwAsCaller(exception);
            end
        end
    end
end