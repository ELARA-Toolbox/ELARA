classdef Workspace
    %% Define the workspace for an optimal control problem
    % The workspace may contain obstacles and workspace boundaries.
    % TODO: Extend workspace objects to shapes such as spheres or ellipsoids.

    properties(SetAccess = protected)
        nObjects (1,1) double

        % Vertices of boxes in the workspace
        boxVertices (8,3,:) double

        % Vector defining if a box is an obstacle or a workspace boundary
        % 0 = obstacle
        % 1 = boundary
        boxTypes (:,1) double
    end

    methods
        %% Methods for adding objects to the workspace

        % TODO: Add further methods for defining boxes or polytopes
        % (e.g. from side lengths and center position/rotation)

        function obj = addBoxFromAxisLimits(obj, boxMin, boxMax, boxType)
            % Add a box to the workspace by specifying the box boundaries
            % in terms of minimal and maximal axis coordinates; i.e.,
            % the box extends from the minimal (x,y,z) values given in
            % boxMin to the maximum values given by boxMax
            arguments
                obj
                boxMin     (3,1) double
                boxMax     (3,1) double
                boxType    (1,1) double
            end

            [X, Y, Z] = ndgrid( ...
                [boxMin(1), boxMax(1)], ...
                [boxMin(2), boxMax(2)], ...
                [boxMin(3), boxMax(3)]);
            verts = [X(:), Y(:), Z(:)];

            obj.boxVertices = cat(3, obj.boxVertices, verts);
            obj.boxTypes(end+1) = boxType;
            obj.nObjects = obj.nObjects + 1;
        end
        function obj = addBoxSideLengths(obj, x, eul, l, boxType)
            % Add a box based on the position of its center, its rotation
            % and its side lengths
            arguments
                obj
                % Box center position
                x     (3,1) double

                % Euler angles specifying the box rotation
                eul     (1,3) double

                % Side lengths (x,y,z)
                l     (1,3) double

                boxType    (1,1) double
            end
            % Compute dimensions of the box relative to its center
            % x+ y+ z+
            % x- y- z-
            db = [l;-l]/2;

            % Construct vertices in local box frame
            verts = [
                db(1,1), db(1,2), db(1,3)
                db(1,1), db(2,2), db(1,3)
                db(1,1), db(2,2), db(2,3)
                db(1,1), db(1,2), db(2,3)
                db(2,1), db(1,2), db(1,3)
                db(2,1), db(2,2), db(1,3)
                db(2,1), db(2,2), db(2,3)
                db(2,1), db(1,2), db(2,3)
                ];

            % Transform into global frame
            g = elara.SE3.matrix(eul2rotm(eul), x);
            for iV = 1:size(verts,1)
                v_i = g*[verts(iV,:).';1];
                verts(iV,:) = v_i(1:3);
            end

            obj.boxVertices = cat(3, obj.boxVertices, verts);
            obj.boxTypes(end+1) = boxType;
            obj.nObjects = obj.nObjects + 1;
        end


        function [polytopeVertices, polytopeTriIndices] = getPolytopeVertexData(obj)
            %% Get Vertices and Vertex Indices of all Polytopes

            % Cell array with vertex matrices
            polytopeVertices = cell(obj.nObjects,1);

            % Cell array with the vertex indices that make up the faces of the convex hull
            polytopeTriIndices = cell(obj.nObjects,1);

            for iPoly = 1:obj.nObjects
                polytopeVertices{iPoly} = obj.boxVertices(:,:,iPoly);
                polytopeTriIndices{iPoly} = convhull(obj.boxVertices(:,:,iPoly));
            end
        end
        function [A, b] = getPolytopeHalfSpaceRepresentation(obj)
            %% Get the half-space representation of all polytopes

            [polytopeVertices, ~] = obj.getPolytopeVertexData;

            % Half-space representation for individual polytopes
            A = cell(obj.nObjects,1);
            b = cell(obj.nObjects,1);
            for iPoly = 1:obj.nObjects
                [A{iPoly}, b{iPoly}] = vert2lcon(polytopeVertices{iPoly});
            end
        end

        function visualize(obj, options)
            %% Draw/visualize workspace
            arguments
                obj (1,1) elara.Workspace

                options.figureName    (1,1) string = "Workspace";
                options.createFigure  (1,1) logical = true;
            end
            if options.createFigure
                elara.visualization.initializeAxes("Name", options.figureName);
            end

            polyColors = lines(2);
            [polytopeVertices, polytopeTriIndices] = obj.getPolytopeVertexData;

            % Individual polytopes
            for iPoly = 1:obj.nObjects
                vertices = polytopeVertices{iPoly};
                color = polyColors(double(obj.boxTypes(iPoly))+1,:);
                trisurf(polytopeTriIndices{iPoly}, ...
                    vertices(:,1), vertices(:,2), vertices(:,3), ...
                    "FaceAlpha", 0.1, "FaceColor", color, ...
                    "EdgeColor", color, "LineWidth", 1.0);
            end
        end

        function [dIntFun, dExtFun] = getSignedDistanceFunctions(workspace, nPoints)
            %% CasADi functions for signed workspace distances
            %
            % This function returns two distance functions:
            %  dIntFun: For objects defining the workspace interior
            %           (object type 1)
            %  dExtFun: For objects defining the workspace exterior (obstacles)
            %           (object type 0)
            arguments
                workspace   (1,1) elara.Workspace

                % Number of points that should be evaluated;
                % equals size of the output
                nPoints     (1,1) double
            end

            indicesExtObjects = find(workspace.boxTypes == 0);
            indicesIntObjects = find(workspace.boxTypes == 1);
            nIntObjects = numel(indicesIntObjects);
            nExtObjects = numel(indicesExtObjects);

            % Get CasADi functions for min and max approximations via LogSumExp
            % (LSE)
            a = 500; % LSE scaling factor
            t = casadi.SX.sym('t', nExtObjects, 1);
            if nExtObjects
                Tlse = logsumexp(t*a)/a;
            else
                Tlse = 0;
            end
            minLSE = casadi.Function('minLSE', {t}, {Tlse});
            t = casadi.SX.sym('t', nIntObjects, 1);
            if nIntObjects
                Tlse = -logsumexp(-t*a)/a;
            else
                Tlse = 0;
            end
            maxLSE = casadi.Function('maxLSE', {t}, {Tlse});

            x_SX = casadi.SX.sym('x', 3, nPoints);

            % Holds the largest distance of each frame to the nearest polytope
            dAnyPolyInt = cell(nPoints, 1);
            dAnyPolyExt = cell(nPoints, 1);

            [A, b] = workspace.getPolytopeHalfSpaceRepresentation;

            % Interior objects
            for iPoint = 1:nPoints
                dIntPolys = cell(length(indicesIntObjects), 1);
                dExtPolys = cell(length(indicesExtObjects), 1);
                if ~isempty(indicesIntObjects)
                    for iPoly = indicesIntObjects'
                        dIntPolys{iPoly} = workspace.polytopeDistance(x_SX(:,iPoint), A{iPoly}, b{iPoly});
                    end
                    dAnyPolyInt{iPoint} = maxLSE(vertcat(dIntPolys{:}));
                end
                if ~isempty(indicesExtObjects)
                    for iPoly = indicesExtObjects'
                        dExtPolys{iPoly} = workspace.polytopeDistance(x_SX(:,iPoint), A{iPoly}, b{iPoly});
                    end
                    dAnyPolyExt{iPoint} = minLSE(vertcat(dExtPolys{:}));
                end
            end
            dIntFun = casadi.Function('dAnyInt', {x_SX}, {vertcat(dAnyPolyInt{:})}, {'x'}, {'dAnyPolyInt'});
            dExtFun = casadi.Function('dAnyExt', {x_SX}, {vertcat(dAnyPolyExt{:})}, {'x'}, {'dAnyPolyExt'});
        end
    end

    methods (Static, Access=private)
        function d = polytopeDistance(x, A, b)
            %% Compute signed distance function for a polytope
            arguments
                % Point to be evaluated
                x (3,1)

                % Half-space representation of the polytope
                A (:,3)
                b (:,1)
            end

            % Get CasADi min-LSE function
            a = 500;
            tA = casadi.SX.sym('t', size(A, 1));
            TlseN = -logsumexp(-tA*a)/a;
            minLSE = casadi.Function('minLSE', {tA}, {TlseN});

            % Evaluate distance
            d = minLSE((b - A*x) ./ vecnorm(A, 2, 2));
        end
    end
end
