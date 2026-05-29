function drawWorkspace(ws, options)
    %% Draw/visualize workspace
    arguments
        ws (1,1) workspaceDefinition

        options.figureName    (1,1) string = "Workspace";
        options.createFigure  (1,1) logical = true;
    end
    if options.createFigure
        fig = init3Dplot("Name", options.figureName);
    else
        fig = gcf;
    end

    nPolytopes = ws.nObjects;
    polyColors = lines(2);

    [pVertices, pTriIndices] = ws.getPolytopeVertexData;

    % Individual polytopes
    for iPoly = 1:nPolytopes
        polyVertices = pVertices{iPoly};
        polyColor = polyColors(double(ws.boxTypes(iPoly))+1,:);
        trisurf(pTriIndices{iPoly}, ...
            polyVertices(:,1), polyVertices(:,2), polyVertices(:,3), ...
            'FaceAlpha', 0.1, 'FaceColor', polyColor, 'EdgeColor', polyColor, 'LineWidth', 1.0);
    end

end