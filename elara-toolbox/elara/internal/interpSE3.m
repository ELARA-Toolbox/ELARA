function gQuery = interpSE3(gInput, sInput, sQuery)
    %% Geometrically correct interpolation of SE3 configurations
    %
    % The input g is an array of SE3 matrices with dimensions
    % (4,4,nNodes,nSteps) that describes a chain of 3D frames in space
    % (e.g., nodes in a discrete geometrically exact beam).
    % 
    % Each frame in the dimension nNodes corresponds to one value in sInput
    % describing its position along the chain.
    % This function interpolates this dimension (nNodes/dimension 3) at the
    % values given by sQuery.
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        % Array of SE3 configuration matrices with dimensions (4,4,nNodes,nSteps)
        gInput  (4,4,:,:) double

        % Vector of frame positions s along the chain (for input
        % discretization)
        sInput  (:,1) double

        % Vector of desired query points (frame positions)
        sQuery  (:,1) double
    end

    % Output array
    gQuery = zeros(4,4,length(sQuery), size(gInput,4));

    for iStep = 1:size(gInput,4)
        
        % Compute discrete "deformations" of the chain, i.e., Lie algebra
        % vectors describing the relative configuration between frames
        xiStep = computeDiscreteDeformations(gInput(:,:,:,iStep), diff(sInput));

        for iSQuery = 1:length(sQuery)

            % Check whether we are the end length
            if sQuery(iSQuery) >= sInput(end)
                % End reached:
                % Simply take last frame configuration
                gQuery(:,:,iSQuery,iStep) = gInput(:,:,end,iStep);
            else
                % We are somewhere along the chain (including s=0):
                % Compute via interpolation

                % Get segment in the original discretization, to which the current s
                % belongs:
                % Get index of first frame (frame position) that is larger than the
                % current position
                iNextNode = find(sInput>sQuery(iSQuery),1);

                % Configuration of the previous node (at the beginning of the segment)
                g0 = gInput(:,:,iNextNode-1,iStep);

                % Get position of the current query point w.r.t. the previous node of
                % the initial discretization
                lRel = sQuery(iSQuery) - sInput(iNextNode-1);

                % Compute configuration
                gQuery(:,:,iSQuery,iStep) = g0*caySE3(lRel * xiStep(:,iNextNode-1));
            end
        end
    end
end

function xi = computeDiscreteDeformations(g, l)
    % Compute the discrete relative "deformations" xi that
    % correspond to the given array of SE3 matrices.
    %  Important: The computed xi values correspond to discrete *updates*,
    %  not discrete *gradients*! I.e., tau(xi) = g_a^{-1}*g_{a+1} holds!
    %  If the discrete "gradients" are required, the result must be divided
    %  by l in the parent function.
    arguments
        % Array of SE3 configuration matrices with dimension (4, 4, nNodes)
        g (4,4,:) double

        % Vector of segment lengths (can vary over the length)
        l (:,1) double
    end

    % Get number of discrete segments
    nSeg = size(g, 3) - 1;

    xi = zeros(6, nSeg);
    for iSeg = 1:nSeg
        xi(:,iSeg) = cayInvSE3( g(:,:,iSeg) \ g(:,:,iSeg+1) ) / l(iSeg);
    end
end
