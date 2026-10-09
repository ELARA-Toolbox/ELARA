function sys = assembleSystem(links, sys)
    %% Assemble an array of ELARA links to the full Multibody System
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        % Array of links defining the system
        links   (1,:) elara.abstract.Link

        % (Empty) elara.abstract.System object, in which the system is stored
        sys     (1,1) elara.abstract.System
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
        "Parent link number is invalid: Number exceeds the number of links in the system." ...
        );

    % Verify that we have only one body with TCP
    assert( sum([links.hasTCP]) <= 1, ...
        "Multiple links specified with TCP. Only one TCP definition allowed." ...
        );


    %% Basic properties

    % Check cantilever flag
    sys.isCantilever = ~links([links.parentLink] == 0).isRigid && links([links.parentLink] == 0).isCantilever;

    sys.nLinks = numel(links);


    %% Build link graph

    % Build graph
    if sys.nLinks > 1
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
    sys.LinkAdjacencyMatrix = full(linkGraph.adjacency);


    %% Build Frame Graph

    frameGraph = digraph();
    sys.linkFrameIndices = zeros(2,sys.nLinks);

    for iLink = 1:sys.nLinks

        % Number of frames of the current body
        if links(iLink).isRigid
            nFrames = 1;
        else
            % Check if current link is the first one and it's a cantilever
            % system
            if isempty(linkGraph.predecessors(iLink)) && sys.isCantilever
                % First node and cantilever -> add one less node
                nFrames = links(iLink).nSegments;
            else
                % Regular beam
                nFrames = links(iLink).nSegments + 1;
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
            iFrmPar  = sys.linkFrameIndices(2, iLinkPar);

            % Add nodes to graph
            frameGraph = frameGraph.addedge( ...
                [iFrmPar,         frameNrsGlob(1:end-1)], ...
                [frameNrsGlob(1), frameNrsGlob(2:end)] ...
                );
        end

        % Store assignment
        sys.linkFrameIndices(1,iLink) = frameNrsGlob(1);
        sys.linkFrameIndices(2,iLink) = frameNrsGlob(end);
    end

    % Get Adjacency matrix
    % Note: Convert from sparse to full matrix to support code generation
    sys.FrameAdjacencyMatrix = full(frameGraph.adjacency);


    %% Compute General System Properties

    % Remove first frame if first link is a cantilever beam
    if sys.isCantilever
        sys.nJoints = sys.nLinks - 1;
    else
        sys.nJoints = sys.nLinks;
    end
    sys.nFrames = frameGraph.numnodes;


    %% Compute Frame DoFs and Graph Structure

    % Prepare maximum length of the system chains to reduce size of the
    % predecessors vector (which is padded with zeros)
    nodeDists = distances(frameGraph);
    maxLength = max(nodeDists(~isinf(nodeDists)));
    sys.frames.ancestors = zeros(maxLength, nFrames);
    sys.frames.parent = zeros(nFrames,1);

    lastIndexq = 1; % Temp variable to compute the system coordinate vector indices
    for iLink = 1:sys.nLinks
        linkFrames = sys.linkFrameIndices(1,iLink):sys.linkFrameIndices(2,iLink);

        for iFrm = 1:length(linkFrames)
            iCurFrame = linkFrames(iFrm);

            sys.frames.linkIndex(iCurFrame) = iLink;

            % System topology: Parent and ancestors
            parent =  frameGraph.predecessors(iCurFrame);
            if ~isempty(parent)
                ancestors = frameGraph.shortestpath(1,parent);
                if ~isempty(ancestors)
                    sys.frames.ancestors(1:length(ancestors),iCurFrame) = ancestors;
                    sys.frames.parent(iCurFrame) = parent;
                end
            end

            % Check if joint is a screw joint or flexible beam joint
            if (isempty(frameGraph.predecessors(iCurFrame)) && sys.isCantilever) || (iFrm > 1)
                % Flexible beam joint
                sys.frames.jointType(iCurFrame) = 2;
                sys.frames.nDof(iCurFrame) = size(links(iLink).Ba, 2);
            else
                % Screw joint
                sys.frames.jointType(iCurFrame) = 1;
                sys.frames.nDof(iCurFrame) = 1;
            end

            % Indices in the system coordinate vector
            sys.frames.qIndices(1,iCurFrame) = lastIndexq;
            sys.frames.qIndices(2,iCurFrame) = lastIndexq + sys.frames.nDof(iCurFrame) - 1;
            lastIndexq = lastIndexq + sys.frames.nDof(iCurFrame);
        end
    end

    sys.nDoF = sum(sys.frames.nDof);


    %% Compute Inertial Properties

    MGen = repmat(eye(6), [1,1,sys.nFrames]);
    sys.frames.x_a  = zeros(3, sys.nFrames);
    sys.frames.g_a  = repmat(eye(4), [1,1,sys.nFrames]);
    sys.frames.m    = zeros(sys.nFrames,1);
    sys.frames.m_a  = zeros(sys.nFrames,1);

    for iLink = 1:sys.nLinks
        linkFrames = sys.linkFrameIndices(1,iLink):sys.linkFrameIndices(2,iLink);

        if links(iLink).isRigid
            %%% Rigid link

            % Inertial properties
            MGen(:,:,linkFrames) = blkdiag(...
                links(iLink).J, ...
                links(iLink).m * eye(3) ...
                );
            sys.frames.m(linkFrames) = links(iLink).m;
            if ~isempty(links(iLink).g_a) && ~isempty(links(iLink).m_a)
                sys.frames.g_a(:,:,linkFrames) = links(iLink).g_a;
                sys.frames.x_a(:,linkFrames)   = links(iLink).g_a(1:3,4);
                sys.frames.m_a(linkFrames)     = links(iLink).m_a;
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
            if isempty(frameGraph.predecessors(linkFrames(1))) && sys.isCantilever
                nodeIndices = 2:(nNodes+1);
            else
                nodeIndices = 1:nNodes;
            end

            % Segment length for the current beam
            l = links(iLink).L / links(iLink).nSegments;

            for iFrm = 1:nNodes
                iCurFrame = linkFrames(iFrm);
                iCurNode = nodeIndices(iFrm);

                if isempty(links(iLink).M_a)
                    MGen(:,:,iCurFrame)          = l*factors(iCurNode)*links(iLink).beamParameters.Mgen;
                    sys.frames.m(iCurFrame) = l*factors(iCurNode)*links(iLink).beamParameters.m;
                else
                    MGen(:,:,iCurFrame) = ...
                        + links(iLink).M_a(:,:,iCurNode) ...
                        + l*factors(iCurNode)*links(iLink).beamParameters.Mgen;
                    sys.frames.m(iCurFrame) = ...
                        + links(iLink).m_a(iCurNode) ...
                        + l*factors(iCurNode)*links(iLink).beamParameters.m;
                    sys.frames.m_a(iCurFrame)     = links(iLink).m_a(iCurNode);
                    sys.frames.g_a(:,:,iCurFrame) = links(iLink).g_a(:,:,iCurNode);
                    sys.frames.x_a(:,iCurFrame)   = links(iLink).g_a(1:3,4,iCurNode);
                end
            end
        end
    end

    % Assign MGen to frames
    if isa(sys, "elara.SystemNum")
        sys.frames.MGen = MGen;
    else
        sys.frames.MGen = squeeze(num2cell(MGen,[1,2]));
    end


    %% Compute joint kinematic and stiffness parameters

    % Vectors of global stiffness and dissipation coefficients
    sys.cSys = zeros(sys.nDoF,1);
    sys.dSys = zeros(sys.nDoF,1);
    sys.qRef = zeros(sys.nDoF,1);
    sys.frames.g_ref  = repmat(eye(4), [1,1,sys.nFrames]);

    for iFrm = 1:sys.nFrames
        iCurLink = sys.frames.linkIndex(iFrm);
        frameQIndices = sys.frames.qIndices(1,iFrm):sys.frames.qIndices(2,iFrm);

        switch sys.frames.jointType(iFrm)
            case 1
                %%% Screw joint

                % Joint kinematics
                sys.frames.X(:,iFrm)       = elara.SE3.AdInv(links(iCurLink).g_J_B) * links(iCurLink).jointAxis;
                sys.frames.g_ref(:,:,iFrm) = links(iCurLink).g_ref;

                % Stiffness and dissipation
                sys.cSys(frameQIndices) = links(iCurLink).c;
                sys.dSys(frameQIndices) = links(iCurLink).d;

            case 2
                %%% Flexible joint

                % Joint kinematics
                l  = links(iCurLink).L / links(iCurLink).nSegments;
                Ba = links(iCurLink).Ba;
                if ~(links(iCurLink).parentLink) && sys.isCantilever
                    segNrLocal = iFrm - sys.linkFrameIndices(1,iCurLink) + 1;
                else
                    segNrLocal = iFrm - sys.linkFrameIndices(1,iCurLink);
                end

                sys.frames.BaPadded(:,1:sys.frames.nDof(iFrm),iFrm) = Ba;
                sys.frames.l(iFrm)   = l;
                sys.frames.xiC(:,iFrm) = links(iCurLink).Bc * links(iCurLink).Bc.' * links(iCurLink).xiRef(:,segNrLocal);

                % Stiffness and dissipation
                sys.cSys(frameQIndices) = l * Ba.' * diag(links(iCurLink).beamParameters.Cgen);
                sys.dSys(frameQIndices) = l * Ba.' * links(iCurLink).beamParameters.d;
                sys.qRef(frameQIndices) = Ba.' * links(iCurLink).xiRef(:,segNrLocal);
            otherwise
                error("Invalid joint type specified.");
        end
    end


    %% Compute Input Assignment

    sys.frames.uIndices = zeros(2, sys.nFrames);

    lastIndexU = 1; % Temp variable to compute the input vector indices
    nTendonsMax = 0; % Maximum number of tendons across all links
    nDisksTotal = 0;
    nDisks = zeros(sys.nLinks,1);
    sDisks = [];
    iDisks = zeros(2, sys.nLinks);
    nSectionsInLink = zeros(sys.nLinks,1);
    dsSections = [];
    nSectionsBetweenDisks = []; % nDisks - 1
    iSectionsBetweenDisks = []; % nDisks - 1
    frameIndexSection = []; % nSections
    lastSection = 1;
    sys.tendonActuationType = zeros(1,sys.nLinks);
    sys.jointActuationType = zeros(1,sys.nLinks);

    for iLink = 1:sys.nLinks
        linkFrames = sys.linkFrameIndices(1,iLink):sys.linkFrameIndices(2,iLink);
        
        % Check if the current link has an actuated lower-pair joint
        hasJointActuation = ...
            (links(iLink).parentLink && links(iLink).jointIsActuated) || ...
            (~links(iLink).parentLink && ~sys.isCantilever && links(iLink).jointIsActuated);
        sys.jointActuationType(iLink) = hasJointActuation;

        % Assign input index for scalar joint actuation to the first
        % frame in the link (which, for rigid links, is the only frame)
        if hasJointActuation
            sys.frames.uIndices(1,linkFrames(1)) = lastIndexU;
            sys.frames.uIndices(2,linkFrames(1)) = lastIndexU;
            lastIndexU = lastIndexU + 1;
        end

        % Check if current link has tendon actuation (for flexible links)
        if ~links(iLink).isRigid && ~isempty(links(iLink).tendonActuation)
            sFramesLink = linspace(0, links(iLink).L, links(iLink).nSegments+1).';

            switch links(iLink).tendonActuation.tendonActuationType
                case 'continuous'
                    sys.tendonActuationType(iLink) = 1;
                    assert(isequal(links(iLink).tendonActuation.sDisks, sFramesLink), ...
                        'Disks and Frame position must be identical for continuous tendon actuation')
                case 'discrete'
                    sys.tendonActuationType(iLink) = 2;
            end

            nTendons = length(links(iLink).tendonActuation.terminationDisks);
            sDisksLink = links(iLink).tendonActuation.sDisks;
            nDisks(iLink) = nDisksTotal + length(sDisksLink);
            sDisks = [sDisks; sDisksLink];
          
            iDisks(1, iLink) = max(iDisks, [], "all") + 1;
            iDisks(2, iLink) = iDisks(1,iLink) + nDisks(iLink) - 1;
            
            sSectionsLink = sort(uniquetol([sDisksLink; sFramesLink]));
            dsSectionsLink = diff(sSectionsLink); 
          %  nSectionsBetweenDisks = [nSectionsInLink length(dsSectionsLink)];
  
            dsSections = [dsSections; dsSectionsLink];
            nSectionsLink(iLink) = length(dsSectionsLink);

            nSectionsBetweenDisksLink = zeros(1,nDisks(iLink));
            iSectionsBetweenDisksLink = zeros(2, nDisks(iLink));
           
            iSectionsBetweenDisksLink(1,1) = lastSection;

            % Check if current link is a cantilever link:
            % Then, the first joint may have cable actuation;
            % otherwise, the second joint is the earliest to may have it
            if ~links(iLink).parentLink && sys.isCantilever
                actuatedFrames = linkFrames;
            else
                actuatedFrames = linkFrames(2:end);
            end

            
            for iSection =1:nSectionsLink(iLink)
                sEndOfSection = sSectionsLink(iSection + 1);
                localFrameIndex = find(sEndOfSection <= sFramesLink(2:end) , 1);
                frameIndexSection = [frameIndexSection; actuatedFrames(localFrameIndex)];

                localDiskIndex = find(sEndOfSection <= sDisksLink(2:end), 1);
                nSectionsBetweenDisksLink(localDiskIndex) = nSectionsBetweenDisksLink(localDiskIndex) + 1; 
                iSectionsBetweenDisksLink(2, localDiskIndex) = iSectionsBetweenDisksLink(1, localDiskIndex) + nSectionsBetweenDisksLink(localDiskIndex)-1 ;
                if localDiskIndex < nDisks(iLink) - 1
                    iSectionsBetweenDisksLink(1, localDiskIndex+1) = iSectionsBetweenDisksLink(2, localDiskIndex) + 1;
                end
            end 
            nSectionsBetweenDisks = [nSectionsBetweenDisks nSectionsBetweenDisksLink];

            
            sys.frames.uIndices(1,actuatedFrames) = lastIndexU;
            sys.frames.uIndices(2,actuatedFrames) = lastIndexU + nTendons - 1;
            lastIndexU = lastIndexU + nTendons;

            nTendonsMax = max([nTendons, nTendonsMax]);
            
            %
            nSectionsInLink(iLink) = length(dsSectionsLink);
            iSectionsBetweenDisks = [iSectionsBetweenDisks iSectionsBetweenDisksLink];
            lastSection = max(iSectionsBetweenDisks, [], "all")+1;
        end

        
    end
    sys.nDisks = nDisks;
    sys.iDisks = iDisks;
    sys.sDisks = sDisks;
    sys.nSectionsInLink = nSectionsInLink;
    sys.nSectionsBetweenDisks = nSectionsBetweenDisks;
    sys.iSectionsBetweenDisks = iSectionsBetweenDisks;
    sys.sSection = dsSections;
    sys.frameIndexSection = frameIndexSection;

    nDisksTotal = max(nDisks);
    % Number of system inputs
    sys.nInputs =  max(sys.frames.uIndices(2,:));


    %% Compute data for tendon actuation

    gBackboneTendon = repmat(eye(4), [1,1,nTendonsMax, nDisksTotal]);
    sys.tendonIsActive = false(nTendonsMax, nDisksTotal);

    for iLink=1:sys.nLinks

        nTendons = length(links(iLink).tendonActuation.terminationDisks);
        iDiskAbsolute = sys.iDisks(1, iLink):sys.iDisks(2, iLink);

        for i = 1:nDisks(iLink)
            iDisk = iDiskAbsolute(i);
            
            gBackboneTendon(:,:,1:nTendons, iDisk) =  links(iLink).tendonActuation.gBackboneTendon(:,:,:,i);
            sys.tendonIsActive(1:nTendons, iDisk) = links(iLink).tendonActuation.terminationDisks >= i; 
        end
    end

    if isa(sys,'elara.SystemNum')
        sys.gBackboneTendon = gBackboneTendon;
    else
        sys.gBackboneTendon = ...
            elara.SE3.matrix2Element(gBackboneTendon);
    end
    %% Store TCP data if TCP is defined

    if any([links.hasTCP])
        iTCPLink = find([links.hasTCP]);
        sys.indexTCPFrame = sys.linkFrameIndices(2, iTCPLink);
        sys.g_B_TCP = links(iTCPLink).g_B_TCP;
    end

end
