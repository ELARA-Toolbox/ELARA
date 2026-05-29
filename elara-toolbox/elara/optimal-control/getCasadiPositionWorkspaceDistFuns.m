function [dIntFun, dExtFun] = getCasadiPositionWorkspaceDistFuns(nPoints, ws)
    %% Casadi function for signed workspace distance functions
    % for multiple points
    %
    % This function returns two distance functions:
    %  dIntFun: For objects defining the workspace interior
    %           (object type 1)
    %  dExtFun: For objects defining the workspace exterior (obstacles)
    %           (object type 0)
    arguments
        % Number of points that should be evaluated;
        % equals size of the output
        nPoints     (1,1) double

        % Workspace object
        ws          (1,1) workspaceDefinition
    end

    indicesExtObjects = find(ws.boxTypes == 0);
    indicesIntObjects = find(ws.boxTypes == 1);
    nIntObjects = numel(indicesIntObjects);
    nExtObjects = numel(indicesExtObjects);

    % Get casadi functions for min and max approximations via LogSumExp
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

    [A, b] = ws.getPTHalfSpaceRepresentation;

    % Interior objects
    for iFrm = 1:nPoints
        dIntPolys = cell(length(indicesIntObjects), 1);
        dExtPolys = cell(length(indicesExtObjects), 1);
        if ~isempty(indicesIntObjects)
            for iPoly = indicesIntObjects'
                dIntPolys{iPoly} = polytopeDistance(x_SX(:,iFrm), A{iPoly}, b{iPoly});
            end
            dAnyPolyInt{iFrm} = maxLSE(vertcat(dIntPolys{:}));
        end
        if ~isempty(indicesExtObjects)
            for iPoly = indicesExtObjects'
                dExtPolys{iPoly} = polytopeDistance(x_SX(:,iFrm), A{iPoly}, b{iPoly});
            end
            dAnyPolyExt{iFrm} = minLSE(vertcat(dExtPolys{:}));
        end
    end
    dIntFun = casadi.Function('dAnyInt', {x_SX}, {vertcat(dAnyPolyInt{:})}, {'x'}, {'dAnyPolyInt'});
    dExtFun = casadi.Function('dAnyExt', {x_SX}, {vertcat(dAnyPolyExt{:})}, {'x'}, {'dAnyPolyExt'});
end

function d = polytopeDistance(x, A, b)
    %% Compute signed distance function for polytope
    arguments
        % Point to be evaluated
        x (3,1)

        % Half space representation of the polytope
        A (:,3)
        b (:,1)
    end
    % Get casadi minLSE function
    a = 500; % LSE scaling factor
    tA = casadi.SX.sym('t', size(A, 1));
    TlseN = -logsumexp(-tA*a)/a;
    minLSE = casadi.Function('minLSE', {tA}, {TlseN});

    % Evaluate distance
    d = minLSE((b - A * x) ./ vecnorm(A, 2, 2));
end