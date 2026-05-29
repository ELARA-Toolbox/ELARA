function p = drawSystemVisProjection(vis, plane, planePos, opts)
    %% Draw 2D Projection of a System Visualization onto a Plane
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        % System visualization
        vis             (1,1) MBSysVisualization

        % Which plane: XY, XZ, YZ
        plane           (1,1) string

        % Coordinate of the plane, on which the projection is drawn
        planePos        (1,1) double

        % Options
        opts.FaceAlpha  (1,1) double = 0.3;
        opts.EdgeAlpha  (1,1) double = 0;
        opts.Color      (3,1) double = ones(3,1)*0.8;
    end

    % Compute homogeneous projection matrix for 3D -> 2D projection
    M = [eye(3), zeros(3,1)];
    switch plane
        case "xy"
            M(3,:) = [0,0,0,planePos];
        case "xz"
            M(2,:) = [0,0,0,planePos];
        case "yz"
            M(1,:) = [0,0,0,planePos];
        otherwise
            error("Plane not implemented");
    end

    p = gobjects(length(vis.linkVis),1);

    for iLink = 1:length(vis.linkVis)
        if isa(vis.linkVis{iLink}, "MBFlexibleLinkVisualization")
            % Get 3D vertices of the beam edges
            verts = vis.linkVis{iLink}.beamVis.hPatchBeam.Vertices();

            % Compute homogeneous points
            vertsH = [verts, ones(size(verts,1),1)];

            Faces = vis.linkVis{iLink}.beamVis.hPatchBeam.Faces;

        elseif isa(vis.linkVis{iLink}, "MBRigidLinkVisualization")
            % Get vertices of the bounding box in 3D space by explictily
            % applying the transformations stored in the graphics objects
            vertsH = (vis.linkVis{iLink}.transf.Matrix ....
                * vis.linkVis{iLink}.transfBB.Matrix ...
                * [vis.linkVis{iLink}.patchBB.Vertices, ones(8,1)].').';

            Faces = vis.linkVis{iLink}.patchBB.Faces;
        end

        % Project onto 2D plane
        verts2D = (M*vertsH.').';

        % Draw as patch
        p(iLink) = patch( ...
            "Faces", Faces, "Vertices", verts2D, ...
            "FaceColor", opts.Color, "EdgeColor", opts.Color, ...
            "FaceAlpha", opts.FaceAlpha, "EdgeAlpha", opts.EdgeAlpha );
    end
end