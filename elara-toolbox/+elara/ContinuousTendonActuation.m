classdef ContinuousTendonActuation < elara.abstract.TendonActuation
    %% Class defining tendon actuation with continuously routed tendons
    % (e.g., inside channels)
    properties (Constant)
        tendonActuationType = "continuous";
    end
    methods
        function obj = ContinuousTendonActuation(LTermination, L, nSegments, x_td_funs, x_td_ds_funs, x_td_dds_funs)
            arguments
                % Vector of lengths, at which the cables terminate (nTendons,1)
                LTermination    (:,1) double {mustBeFinite,mustBePositive}

                % Overall beam length
                L               (1,1) double {mustBeFinite,mustBePositive}

                % Number of discrete segments of the beam
                nSegments       (1,1) double {mustBeFinite,mustBePositive}

                % Cell array of function handles that define the cable path (cable
                % position relative to the backbone/centerline) over beam length
                % dimensions (nTendons, 1)
                % The functions must have the form
                %           [x;y;z] = f(s)
                % where f : R -> R^3 returns the cable path at s
                x_td_funs        (:,1) cell

                % First and second derivatives of the cable path functions w.r.t. s
                x_td_ds_funs     (:,1) cell = {}
                x_td_dds_funs    (:,1) cell = {}

            end

            % Compute disc arc-length locations (at the beam nodes)
            sDisks = linspace(0,L,nSegments+1).';

            if isempty(x_td_ds_funs)
                s = sym("s");
                x_td_ds_funs = cellfun( ...
                    @(x) matlabFunction( diff(x(s), s, 1), "Vars", s ), ...
                    x_td_funs, ...
                    'UniformOutput', false);
            end
            if isempty(x_td_dds_funs)
                s = sym("s");
                x_td_dds_funs = cellfun( ...
                    @(x) matlabFunction( diff(x(s), s, 2), "Vars", s ), ...
                    x_td_funs, ...
                    'UniformOutput', false);
            end

            % Make sure function array sizes match
            assert(length(x_td_funs) == length(x_td_ds_funs));
            assert(length(x_td_funs) == length(x_td_dds_funs));
            assert(length(x_td_funs) == length(LTermination));

            obj.sDisks = sDisks;

            nDisks  = length(sDisks);
            nTendons = length(x_td_funs);

            % Compute relative transformation / position of the tendon path
            % at the nodes
            obj.gBackboneTendon = repmat(eye(4), [1,1,nTendons,nDisks]);
            % x_cm = zeros(3, nNodes, nTendons);

            obj.terminationDisks = zeros(nTendons, 1);

            for iTendon = 1:nTendons
                % Tendon positions
                xBackboneTendons = cell2mat( ...
                    arrayfun( x_td_funs{iTendon}, sDisks , ...
                    'UniformOutput', false).' ...
                    );

                %%% Tangential tendon path rotation matrices (Frenet-Serret
                %%% frames)
                R = zeros(3, 3, nDisks);

                % Get function handles
                f_ds  = x_td_ds_funs{iTendon};
                f_dds = x_td_dds_funs{iTendon};

                % Compute basis vectors of the Frenet-Serret frame
                for iDisk = 1:nDisks
                    sDisk = sDisks(iDisk);
                    % Tangent vector t
                    t = f_ds(sDisk);% / norm(f_ds(lNodes(iN)));
                    t(3) = 1;
                    t = t / norm(t);

                    % Normal direction: component of the second derivative
                    % perpendicular to the unit tangent.
                    a = f_dds(sDisk);
                    n = a - t * (t.' * a);

                    % At zero curvature, choose a perpendicular reference direction.
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
                obj.gBackboneTendon(1:3,4, iTendon, :) = xBackboneTendons;
                obj.gBackboneTendon(1:3,1:3, iTendon, :) = R;

                % Compute disk at which each tendon ends
                [~, obj.terminationDisks(iTendon)] = min(abs(sDisks - LTermination(iTendon)));

                % For short tendons (shorter than the segment length), use
                % the second disk to avoid zero-length tendons
                obj.terminationDisks(iTendon) = max(obj.terminationDisks(iTendon), 2);
            end

            % Validate computed transformations
            try
                elara.internal.validation.mustBeSE3Matrix(obj.gBackboneTendon);
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
