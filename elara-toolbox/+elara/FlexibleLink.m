classdef FlexibleLink < elara.internal.Link
    % Class defining a Flexible Link in a multibody system
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    properties
        %% General properties

        % Applicable only for the first link:
        % Defines whether or not the beam has a joint or is fixed
        isCantilever    (1,1) logical

        %% Flexible Link properties

        % Nr. of segments of the discretized beam
        nSeg        (1,1) double {mustBeNonnegative}

        % Length of the beam (from first to last cross section)
        L           (1,1) double {mustBeNonnegative}

        % Selection matrices for reduced (constrained) beam models
        Ba          (6,:) double
        Bc          (6,:) double

        % Beam material
        beamPars    (1,1) elara.BeamParams

        % Discrete deformations in the reference configuration (6,nSeg)
        xiRef       (6,:) double

        %%% For continuum manipulators
        % Configuration object for cable actuation
        tendonActuation (1,1) elara.TendonActuation

    end
    properties (Constant)
        isRigid = false;
    end
    methods
        function validateProperties(link)
            %% Validate the link's properties
            arguments
                link (1,1) elara.FlexibleLink
            end

            %%% Validate properties of flexible links
            % Check for valid segment number
            assert(link.nSeg > 1, ...
                "Nr. of segments for flexible links must be specified and positive.");

            % Check that inertia tensor, mass and COM transf. arrays for
            % attached masses are either all empty or have the
            % correct dimensions
            assert( size(link.M_a, 3) == size(link.m_a, 2), ...
                ['Inconsistent array sizes for masses and inertia tensors of attached bodies. ' ...
                'To include attached bodies, both arrays must have values and the correct dimensions.'] ...
                );
            assert( size(link.M_a, 3) == size(link.g_a, 3), ...
                ['Inconsistent array sizes for position vectors and inertia tensors of attached bodies. ' ...
                'To include attached bodies, both arrays must have values and the correct dimensions.'] ...
                );

            % Check properties of selection matrices
            r = size(link.Ba, 2); % Nr. of allowed deformation modes
            assert( size(link.Bc, 2) == 6-r, ...
                "Selection matrix Bc does not have the right nr. of columns." );
            assert( all(link.Ba .' * link.Ba == eye(r), "all"), ...
                "Selection matrix Ba does not have the right form." );
            assert( all(link.Bc .' * link.Bc == eye(6-r), "all"), ...
                "Selection matrix Bc does not have the right form." );
            assert( all(link.Bc .' * link.Ba == zeros(6-r,r), "all"), ...
                "Selection matrices Ba and Bc are not complementary." );
        end
        function linkVis = getLinkVisualization(link, varargin)
            %% Initialize and return link visualization object
            linkVis = elara.visualization.FlexibleLinkVisualization(link, varargin{:});
        end
    end
end