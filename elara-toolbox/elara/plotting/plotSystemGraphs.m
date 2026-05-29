function [fhLinkGraph, fhFrameGraph] = plotSystemGraphs(MBSys)
    %% Plot Topology Graphs of a MB System

    arguments
        MBSys (1,1) MBSystem
    end

    linkGraph  = digraph(MBSys.AdjMatrixLinkGraph);
    frameGraph = digraph(MBSys.AdjMatrixFrameGraph);

    % Link colors
    linkColors = lines(MBSys.nLinks);


    %% Plot Link Graph

    fhLinkGraph = figure("Name","LinkGraph");
    h = plot(linkGraph, "Interpreter","latex");
    title('System Topology: Link Graph', "Interpreter", "latex");

    % Set Color for individual bodies
    for iLink = 1:linkGraph.numnodes
        h.highlight(iLink,'NodeColor',linkColors(iLink,:), "MarkerSize", 5);
    end


    %% Plot Frame Graph

    % Determine if the bodies are rigid or flexible
    isRigidLink = ~(MBSys.linkFrameIndices(2,:)-MBSys.linkFrameIndices(1,:));

    fhFrameGraph = figure("Name","FrameGraph");
    ax = axes(fhFrameGraph);
    hold on
    h = plot(ax, frameGraph);
    h.Interpreter = "latex";
    h.HandleVisibility = "off";

    for iLink = 1:linkGraph.numnodes
        linkFrames = MBSys.linkFrameIndices(1,iLink):MBSys.linkFrameIndices(2,iLink);

        % Set Color for individual bodies
        path = shortestpath(frameGraph,linkFrames(1),linkFrames(end));
        h.highlight(path,'NodeColor',linkColors(iLink,:),'EdgeColor', linkColors(iLink,:))

        % Properties for inner beam nodes
        h.highlight(linkFrames(2:end-1), ...
            "MarkerSize", 4, "Marker","square");

        % Properties for rigid bodies and start/end nodes of beams
        h.highlight(linkFrames([1,end]), "MarkerSize", 5);

        % Create dummy plot for legend
        if isRigidLink(iLink)
            typeString = "rigid";
        else
            typeString = "flexible";
        end
        plot(ax, nan,nan, "Color", linkColors(iLink,:), ...
            "MarkerFaceColor", linkColors(iLink,:), ...
            "Marker", "o", ...%"LineWidth", 1 ,...
            "DisplayName", sprintf("Body %d (%s)", iLink, typeString));
    end
    ax.XTick = [];
    ax.YTick = [];
    title('System Topology: Frame Graph', "Interpreter","latex");
    legend("Interpreter", "latex");
    box on;

end