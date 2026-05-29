function MBSys = assembleMBSystem(links, MBSys)
    %% Assemble an array of MB links to the full MB System
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        % Array of links defining the system
        links   (1,:) MBLinkDefinition

        % (Empty) MBSystem object, in which the system is stored
        MBSys   (1,1) MBSystem
    end


    %% Input data validation

    % Validate properties of individual links
    for iLink = 1:numel(links)
        links(iLink).validateProperties;
    end

    % Check that we have only one body with parent 0
    assert( isscalar(links([links.parentLink] == 0)), ...
        "Multiple links specified with no parent." ...
        );

    % Check that all parent numbers are valid link numbers
    assert( all([links.parentLink] < numel(links)), ...
        "Parent link number not valid: Number exceed number of links in the system." ...
        );

    % Verify that we have only one body with TCP
    assert( sum([links.hasTCP]) <= 1, ...
        "Multiple links specified with TCP. Only one TCP definition allowed." ...
        );


    %% Basic properties

    % Check cantilever flag
    MBSys.isCantilever = ~links([links.parentLink] == 0).isRigid && links([links.parentLink] == 0).isCantilever;

    MBSys.nLinks = numel(links);


    %% Build link graph

    % Build graph
    if MBSys.nLinks > 1
        % Indices of bodies with non-zero parent
        lowerBodies = find([links.parentLink]);

        linkGraph = digraph( ...
            [links(lowerBodies).parentLink], ...
            lowerBodies ...
            );
    else
        linkGraph = digraph;
        linkGraph = linkGraph.addnode(1);
    end

    % Check topology
    assert(~linkGraph.hascycles, "System topology contains cycles. Please make sure the system has open-tree structure.")

    % Store adjacency matrix
    % Note: Convert from sparse to full matrix to support code generation
    MBSys.AdjMatrixLinkGraph = full(linkGraph.adjacency);


    %% Build Frame Graph

    frameGraph = digraph();
    MBSys.linkFrameIndices = zeros(2,MBSys.nLinks);

    for iLink = 1:MBSys.nLinks

        % Nr. of frames of the current body
        if links(iLink).isRigid
            nFrames = 1;
        else
            % Check if current link is the first one and it's a cantilever
            % system
            if isempty(linkGraph.predecessors(iLink)) && MBSys.isCantilever
                % First node and cantilever -> add one less node
                nFrames = links(iLink).nSeg;
            else
                % Regular beam
                nFrames = links(iLink).nSeg + 1;
            end
        end

        % Local frame numbers
        frameNrsLocal = 1:nFrames;

        % Global frame numbers
        frameNrsGlob = frameGraph.numnodes + frameNrsLocal;

        %%% Add frames to graph
        if isempty(linkGraph.predecessors(iLink))
            % First link/no predecessor

            if links(iLink).isRigid
                % Rigid first body: No edge, just individual node
                frameGraph = frameGraph.addnode(1);
            else
                % Flexible first body: Add beam nodes
                frameGraph = frameGraph.addedge( ...
                    frameNrsGlob(1:end-1), ...
                    frameNrsGlob(2:end) ...
                    );
            end
        else
            % Get parent link of current link
            iLinkPar = linkGraph.predecessors(iLink);

            % Get parent frame of first frame of the current link
            % = last frame of parent link
            iFrmPar  = MBSys.linkFrameIndices(2, iLinkPar);

            % Add nodes to graph
            frameGraph = frameGraph.addedge( ...
                [iFrmPar,         frameNrsGlob(1:end-1)], ...
                [frameNrsGlob(1), frameNrsGlob(2:end)] ...
                );
        end

        % Store assignment
        MBSys.linkFrameIndices(1,iLink) = frameNrsGlob(1);
        MBSys.linkFrameIndices(2,iLink) = frameNrsGlob(end);
    end

    % Get Adjacency matrix
    % Note: Convert from sparse to full matrix to support code generation
    MBSys.AdjMatrixFrameGraph = full(frameGraph.adjacency);


    %% Compute General System Properties

    % Remove first frame if first link is a cantilever beam
    if MBSys.isCantilever
        MBSys.nJoints = MBSys.nLinks - 1;
    else
        MBSys.nJoints = MBSys.nLinks;
    end
    MBSys.nFrames = frameGraph.numnodes;


    %% Compute Frame DoFs and Graph Structure

    % Prepare maximum length of the system chains to reduce size of the
    % predecessors vector (which is padded with zeros)
    nodeDists = distances(frameGraph);
    maxLength = max(nodeDists(~isinf(nodeDists)));
    MBSys.frameData.ancestors = zeros(maxLength, nFrames);
    MBSys.frameData.parent = zeros(nFrames,1);

    lastIndexq = 1; % Temp variable to compute the system coordinate vector indices
    for iLink = 1:MBSys.nLinks
        linkFrames = MBSys.linkFrameIndices(1,iLink):MBSys.linkFrameIndices(2,iLink);

        for iFrm = 1:length(linkFrames)
            iCurFrame = linkFrames(iFrm);

            MBSys.frameData.linkIndex(iCurFrame) = iLink;

            % System topology: Parent and ancestors
            parent =  frameGraph.predecessors(iCurFrame);
            if ~isempty(parent)
                ancestors = frameGraph.shortestpath(1,parent);
                if ~isempty(ancestors)
                    MBSys.frameData.ancestors(1:length(ancestors),iCurFrame) = ancestors;
                    MBSys.frameData.parent(iCurFrame) = parent;
                end
            end

            % Check if joint is a screw joint or flexible beam joint
            if (isempty(frameGraph.predecessors(iCurFrame)) && MBSys.isCantilever) || (iFrm > 1)
                % Flexible beam joint
                MBSys.frameData.jointType(iCurFrame) = 2;
                MBSys.frameData.nDof(iCurFrame) = size(links(iLink).Ba, 2);
            else
                % Screw joint
                MBSys.frameData.jointType(iCurFrame) = 1;
                MBSys.frameData.nDof(iCurFrame) = 1;
            end

            % Indices in the system coordinate vector
            MBSys.frameData.qIndices(1,iCurFrame) = lastIndexq;
            MBSys.frameData.qIndices(2,iCurFrame) = lastIndexq + MBSys.frameData.nDof(iCurFrame) - 1;
            lastIndexq = lastIndexq + MBSys.frameData.nDof(iCurFrame);
        end
    end

    MBSys.nDoF = sum(MBSys.frameData.nDof);


    %% Compute Inertial Properties

    MGen = repmat(eye(6), [1,1,MBSys.nFrames]);
    MBSys.frameData.x_a  = zeros(3, MBSys.nFrames);
    MBSys.frameData.g_a  = repmat(eye(4), [1,1,MBSys.nFrames]);
    MBSys.frameData.m    = zeros(MBSys.nFrames,1);
    MBSys.frameData.m_a  = zeros(MBSys.nFrames,1);

    for iLink = 1:MBSys.nLinks
        linkFrames = MBSys.linkFrameIndices(1,iLink):MBSys.linkFrameIndices(2,iLink);

        if links(iLink).isRigid
            %%% Rigid link

            % Inertial properties
            MGen(:,:,linkFrames) = blkdiag(...
                links(iLink).J, ...
                links(iLink).m * eye(3) ...
                );
            MBSys.frameData.m(linkFrames) = links(iLink).m;
            if ~isempty(links(iLink).g_a) && ~isempty(links(iLink).m_a)
                MBSys.frameData.g_a(:,:,linkFrames) = links(iLink).g_a;
                MBSys.frameData.x_a(:,linkFrames)   = links(iLink).g_a(1:3,4);
                MBSys.frameData.m_a(linkFrames)     = links(iLink).m_a;
            end
        else
            %%% Flexible link

            % Get factors for node masses
            nNodes = length(linkFrames);
            factors = [0.5;ones(nNodes-1,1);0.5];

            % If beam is the first link in the system and it's a cantilever
            % beam, the factor of the first node is one

            % Get local node indices that include the first node;
            % for cantilever beams, the node index of the first frame is 2
            if isempty(frameGraph.predecessors(linkFrames(1))) && MBSys.isCantilever
                nodeIndices = 2:(nNodes+1);
                %factors = [ones(nNodes-1,1);0.5];
            else
                nodeIndices = 1:nNodes;
                %factors = [0.5;ones(nNodes-2,1);0.5];
            end

            % Segment length for the current beam
            l = links(iLink).L / links(iLink).nSeg;

            for iFrm = 1:nNodes
                iCurFrame = linkFrames(iFrm);
                iCurNode = nodeIndices(iFrm);

                if isempty(links(iLink).M_a)
                    MGen(:,:,iCurFrame)          = l*factors(iCurNode)*links(iLink).beamPars.Mgen;
                    MBSys.frameData.m(iCurFrame) = l*factors(iCurNode)*links(iLink).beamPars.m;
                else
                    MGen(:,:,iCurFrame) = ...
                        + links(iLink).M_a(:,:,iCurNode) ...
                        + l*factors(iCurNode)*links(iLink).beamPars.Mgen;
                    MBSys.frameData.m(iCurFrame) = ...
                        + links(iLink).m_a(iCurNode) ...
                        + l*factors(iCurNode)*links(iLink).beamPars.m;
                    MBSys.frameData.m_a(iCurFrame)     = links(iLink).m_a(iCurNode);
                    MBSys.frameData.g_a(:,:,iCurFrame) = links(iLink).g_a(:,:,iCurNode);
                    MBSys.frameData.x_a(:,iCurFrame)   = links(iLink).g_a(1:3,4,iCurNode);
                end
            end
        end
    end

    % Assign MGen to framedata
    if isa(MBSys, "MBSystemNum")
        MBSys.frameData.MGen = MGen;
    else
        MBSys.frameData.MGen = squeeze(num2cell(MGen,[1,2]));
    end


    %% Compute joint kinematic and stiffness parameters

    % Vectors of global stiffness and dissipation coefficients
    MBSys.cSys = zeros(MBSys.nDoF,1);
    MBSys.dSys = zeros(MBSys.nDoF,1);
    MBSys.qRef = zeros(MBSys.nDoF,1);
    MBSys.frameData.g_ref  = repmat(eye(4), [1,1,MBSys.nFrames]);

    for iFrm = 1:MBSys.nFrames
        iCurLink = MBSys.frameData.linkIndex(iFrm);
        frameQIndices = MBSys.frameData.qIndices(1,iFrm):MBSys.frameData.qIndices(2,iFrm);

        switch MBSys.frameData.jointType(iFrm)
            case 1
                %%% Screw joint

                % Joint kinematics
                MBSys.frameData.X(:,iFrm)       = lAdSE3Inv(links(iCurLink).g_J_B) * links(iCurLink).jointAxis;
                MBSys.frameData.g_ref(:,:,iFrm) = links(iCurLink).g_ref;

                % Stiffness and dissipation
                MBSys.cSys(frameQIndices) = links(iCurLink).c;
                MBSys.dSys(frameQIndices) = links(iCurLink).d;

            case 2
                %%% Flexible joint

                % Joint kinematics
                l  = links(iCurLink).L / links(iCurLink).nSeg;
                Ba = links(iCurLink).Ba;
                if ~(links(iCurLink).parentLink) && MBSys.isCantilever
                    segNrLocal = iFrm - MBSys.linkFrameIndices(1,iCurLink) + 1;
                else
                    segNrLocal = iFrm - MBSys.linkFrameIndices(1,iCurLink);
                end

                MBSys.frameData.BaPadded(:,1:MBSys.frameData.nDof(iFrm),iFrm) = Ba;
                MBSys.frameData.l(iFrm)   = l;
                MBSys.frameData.xiC(:,iFrm) = links(iCurLink).Bc * links(iCurLink).Bc.' * links(iCurLink).xiRef(:,segNrLocal);

                % Stiffness and dissipation
                MBSys.cSys(frameQIndices) = l * Ba.' * diag(links(iCurLink).beamPars.Cgen);
                MBSys.dSys(frameQIndices) = l * Ba.' * links(iCurLink).beamPars.d;
                MBSys.qRef(frameQIndices) = Ba.' * links(iCurLink).xiRef(:,segNrLocal);
            otherwise
                error("Invalid joint type specified.");
        end
    end


    %% Compute Input Assignment

    MBSys.frameData.uIndices = zeros(2, MBSys.nFrames);

    lastIndexU = 1; % Temp variable to compute the input vector indices
    nCablesMax = 0; % Temp variable holding the max. nr. of cables of all links

    for iLink = 1:MBSys.nLinks
        linkFrames = MBSys.linkFrameIndices(1,iLink):MBSys.linkFrameIndices(2,iLink);

        % Check if the current link has an actuated lower-pair joint
        hasJointActuation = ...
            (links(iLink).parentLink && links(iLink).isActuated) || ...
            (~links(iLink).parentLink && ~MBSys.isCantilever && links(iLink).isActuated);

        % Assign input index for scalar joint actuation to the first
        % frame in the link (which, for rigid links, is the only frame)
        if hasJointActuation
            MBSys.frameData.uIndices(1,linkFrames(1)) = lastIndexU;
            MBSys.frameData.uIndices(2,linkFrames(1)) = lastIndexU;
            lastIndexU = lastIndexU + 1;
        end

        % Check if current link has cable actuation (for flexible links)
        if ~links(iLink).isRigid && ~isempty(links(iLink).cableConfig.x_m_funs)

            nCables = length(links(iLink).cableConfig.x_m_funs);

            % Check if current link is a cantilever link:
            % Then, the first joint may have cable actuation;
            % otherwise, the second joint is the earliest to may have it
            if ~links(iLink).parentLink && MBSys.isCantilever
                actuatedFrames = linkFrames;
            else
                actuatedFrames = linkFrames(2:end);
            end

            MBSys.frameData.uIndices(1,actuatedFrames) = lastIndexU;
            MBSys.frameData.uIndices(2,actuatedFrames) = lastIndexU + nCables - 1;
            lastIndexU = lastIndexU + nCables - 1;

            nCablesMax = max([nCables, nCablesMax]);
        end
    end

    % Nr. of system inputs
    MBSys.nInputs =  max(MBSys.frameData.uIndices(2,:));


    %% Compute data for cable actuation

    g_cm = zeros(4,4,2,MBSys.nFrames, nCablesMax);

    for iFrm = 1:MBSys.nFrames
        % Check whether the joint is a beam joint and if it's actuated
        % (=non-zero input index)
        if MBSys.frameData.jointType(iFrm) == 2 && MBSys.frameData.uIndices(1,iFrm)
            iCurLink = MBSys.frameData.linkIndex(iFrm);

            % Get arc lengths of the beam nodes of the current link
            linkFrameIndices = MBSys.linkFrameIndices(1,iCurLink):MBSys.linkFrameIndices(2,iCurLink);
            sLinkFrames = [0; cumsum(MBSys.frameData.l(linkFrameIndices))];

            % Get cable actuation data for current link
            [g_m, termNodes] = links(iCurLink).cableConfig.getNodeData(sLinkFrames);

            % Get local node nr. from iN = 0, ... , nSeg
            if ~(links(iCurLink).parentLink) && MBSys.isCantilever
                nodeIndexLocal = iFrm - MBSys.linkFrameIndices(1,iCurLink) + 2;
            else
                nodeIndexLocal = iFrm - MBSys.linkFrameIndices(1,iCurLink) + 1;
            end
            % Assign cable positions to frames
            for iC = 1:length(links(iLink).cableConfig.x_m_funs)
                if segNrLocal < termNodes(iC)
                    g_cm(:,:,1,iFrm,iC) = g_m(:,:,nodeIndexLocal-1,iC);
                    g_cm(:,:,2,iFrm,iC) = g_m(:,:,nodeIndexLocal,iC);
                end
            end
        end
    end

    % Assign to framedata
    if isa(MBSys, "MBSystemNum")
        MBSys.frameData.g_cm = g_cm;
    else
        MBSys.frameData.g_cm = SE3MatArray2SE3Array(g_cm);
    end

    %% Store TCP data if TCP is defined

    if any([links.hasTCP])
        iTCPLink = find([links.hasTCP]);
        MBSys.indexTCPFrame = MBSys.linkFrameIndices(2, iTCPLink);
        MBSys.g_B_TCP = links(iTCPLink).g_B_TCP;
    end

end
