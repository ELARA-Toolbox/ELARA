classdef TendonActuation
    %% Class to define the actuation properties of cable actuated flexible links
    % used to model cable-actuated continuum manipulators
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        % Vector of lengths, at which the cables terminate (nCables,1)
        LTermination (:,1) double

        % Cell array of function handles that define the cable path (cable
        % position relative to the backbone/centerline) over beam length
        % dimensions (nCables, 1)
        % The functions must have the form
        %           [x;y;z] = f(s)
        % where f : R -> R^3 returns the cable path at s
        x_td_funs        (:,1) cell

        % First and second derivatives of the cable path functions w.r.t. s
        x_td_ds_funs     (:,1) cell
        x_td_dds_funs    (:,1) cell
    end
    methods
        function obj = getSymbolicPathDerivatives(obj)
            %% Compute derivatives of the cable path functions symbolically
            s = sym("s");
            obj.x_td_ds_funs = cellfun( ...
                @(x) matlabFunction( diff(x(s), s, 1), "Vars", s ), ...
                obj.x_td_funs, ...
                'UniformOutput', false);
            obj.x_td_dds_funs = cellfun( ...
                @(x) matlabFunction( diff(x(s), s, 2), "Vars", s ), ...
                obj.x_td_funs, ...
                'UniformOutput', false);
        end

        function [g_cm, termNodes, x_cm] = getNodeData(obj, sNodes)
            %% Compute discrete cable node configurations from function handles
            arguments(Input)
                obj     (1,1) elara.TendonActuation

                % Beam length
                sNodes       (:,1) double {mustBeNonnegative}
            end
            arguments(Output)
                % SE3 transformation corresponding to the (tangential)
                % Frenet-Serret frame at the  intersection of the cable
                % path with the cross-section plane; relative to the
                % backbone
                % dimensions (4,4,nNodes,nCables)
                g_cm        (4,4,:,:) double {mustBeSE3MatrixArray}

                % Vector with nodes, at which the cables terminate
                % dimensions (nCables, 1)
                termNodes   (:,1) double

                % Vector with relative positions of the cables w.r.t. the
                % backbone, dimensions (3, nNodes, nCables)
                x_cm        (3,:,:) double
            end

            % Make sure function array sizes match
            assert(length(obj.x_td_funs) == length(obj.x_td_ds_funs));
            assert(length(obj.x_td_funs) == length(obj.x_td_dds_funs));
            assert(length(obj.x_td_funs) == length(obj.LTermination));

            % Vector with node arc length positions
            %l = L / nSeg;
            %sNodes = 0:l:L;

            nNodes  = length(sNodes);
            nCables = length(obj.x_td_funs);

            % Compute relative transformation / position of the cable path
            % at the nodes
            g_cm = zeros(4,4,nNodes,nCables);
            x_cm = zeros(3, nNodes, nCables);
            termNodes = zeros(nCables, 1);
            for iC = 1:nCables
                % Cable positions
                x_cm(:,:,iC) = cell2mat( ...
                    arrayfun( obj.x_td_funs{iC}, sNodes , ...
                    'UniformOutput', false).' ...
                    );

                %%% Tangential cable path rotation matrices (Frenet-Serret
                %%% frames)
                R = zeros(3, 3, nNodes);

                % Get function handles
                f_ds  = obj.x_td_ds_funs{iC}; %diff(obj.x_td_funs{iC}(s), s, 1);
                f_dds = obj.x_td_dds_funs{iC}; %diff(obj.x_td_funs{iC}(s), s, 2);

                % Compute basis vectors of the Frenet-Serret frame
                for iN = 1:nNodes
                    % Tangent vector t
                    t = f_ds(sNodes(iN));% / norm(f_ds(lNodes(iN)));
                    t(3) = 1;
                    t = t / norm(t);

                    % Normal vector n
                    if any(f_dds(sNodes(iN)))
                        n = f_dds(sNodes(iN)) / norm(f_dds(sNodes(iN)));
                    else
                        n = [1;0;0];
                    end
                    % Bi-normal vector b
                    b = cross(n, t);

                    % Store in rotation matrix; z-axis is aligned with
                    % tangent vector
                    R(:,:,iN) = [b,n,t];
                end

                % Cable mount transformations
                g_cm(:,:,:,iC) = repmat(eye(4), [1,1,nNodes]);
                g_cm(1:3, 4, :, iC)   = x_cm(:,:,iC);
                g_cm(1:3, 1:3, :, iC) = R;

                % Compute nearest node to the termination point of the cables
                % get closest node in nodePos
                [~, termNodes(iC)] = min(abs(sNodes - obj.LTermination(iC)));
            end
        end
    end
end
