classdef workspaceDefinition
    %% Define the Workspace for an Optimal Control Problem
    % which may consist of obstacles / and or workspace boundaries

    %%% TODO
    % For use with homotopy, maybe add subclass for one obstacle
    % with initial and final configuration (based on homotopy parameter);
    % this class then evaluates the child's config etc.
    %%% Maybe do this in another class

    %%% TODO
    % Maybe also extend to other objects, e.g., spheres or ellipsoids

    properties
        %
    end
    properties(SetAccess = protected)
        nObjects (1,1) double

        % Vertices of boxes in the workspace
        boxVerts (8,3,:) double

        % Vector defining if a box is an obstacle or a workspace boundary
        % 0 = obstacle
        % 1 = boundary
        boxTypes (:,1) double
    end

    methods
        %% Methods to add Objects to the Workspace

        %%% TODO: Add other methods to add boxes / polytopes
        % (e.g. from side lengths and center position/rotation)

        function obj = addBoxFromAxisLimits(obj, box_min, box_max, box_type)
            % Add a box to the workspace by specifying the box boundaries
            % in terms of minimal and maximal axis coordinates; i.e.,
            % the box extends from the minimal (x,y,z) values given in
            % box_min to the maximum values given box_max
            arguments
                obj
                box_min     (3,1) double
                box_max     (3,1) double
                box_type    (1,1) double
            end

            [X, Y, Z] = ndgrid( ...
                [box_min(1), box_max(1)], ...
                [box_min(2), box_max(2)], ...
                [box_min(3), box_max(3)]);
            verts = [X(:), Y(:), Z(:)];

            obj.boxVerts = cat(3, obj.boxVerts, verts);
            obj.boxTypes(end+1) = box_type;
            obj.nObjects = obj.nObjects + 1;
        end
        function obj = addBoxSideLengths(obj, x, eul, l, box_type)
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

                box_type    (1,1) double
            end
            % Dimensions of the bounding box, measured form g_bbox
            % x+ y+ z+
            % x- y- z-
            %bBoxSize    (2,3) double


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
            g = SE3Matrix(eul2rotm(eul), x);
            for iV = 1:size(verts,1)
                v_i = g*[verts(iV,:).';1];
                verts(iV,:) = v_i(1:3);
            end

            obj.boxVerts = cat(3, obj.boxVerts, verts);
            obj.boxTypes(end+1) = box_type;
            obj.nObjects = obj.nObjects + 1;
        end


        function [polytopeVertices, polytopeTriIndices] = getPolytopeVertexData(obj)
            %% Get Vertices and Vertex Indices of all Polytopes

            % Cell array with vertex matrices
            polytopeVertices = cell(obj.nObjects,1);

            % Cell array with the vertex indices that make up the faces of the convex hull
            polytopeTriIndices = cell(obj.nObjects,1);

            for iPoly = 1:obj.nObjects
                polytopeVertices{iPoly} = obj.boxVerts(:,:,iPoly);
                polytopeTriIndices{iPoly} = convhull(obj.boxVerts(:,:,iPoly));
            end
        end
        function [A, b] = getPTHalfSpaceRepresentation(obj)
            %% Get the half-space representation of all polytopes

            [polytopeVertices, ~] = obj.getPolytopeVertexData;

            % Half-space representation for individual polytopes
            A = cell(obj.nObjects,1);
            b = cell(obj.nObjects,1);
            for iPoly = 1:obj.nObjects
                [A{iPoly}, b{iPoly}] = vert2lcon(polytopeVertices{iPoly});
            end
        end
    end
end