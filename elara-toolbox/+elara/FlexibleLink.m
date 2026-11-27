classdef FlexibleLink < elara.abstract.Link
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
        isCantilever    (1,1) logical = false;

        %% Flexible Link properties

        % Number of segments in the discretized beam
        nSegments   (1,1) double {mustBePositive} = 5;

        % Length of the beam (from first to last cross section)
        L           (1,1) double {mustBePositive} = 1;

        % Complementary selection matrices for reduced (constrained) beam models
        Ba          (6,:) double = eye(6);
        Bc          (6,:) double = zeros(6,0);

        % Beam material
        beamParameters      (1,1) elara.BeamParameters

        % Discrete deformations in the reference configuration (6,nSegments)
        xiRef               (6,:) double = repmat([0;0;0;0;0;1], [1,5]);

        %%% For continuum manipulators
        % Configuration object for cable actuation
        tendonActuation     (1,:)  elara.abstract.TendonActuation = elara.ContinuousTendonActuation.empty

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
            % Require at least two segments for the beam discretization
            assert(link.nSegments > 1, ...
                'elara:InvalidBeamDiscretization', ...
                "A flexible link must have at least two segments.");

            % Check dimension of tendonActuation property
            assert(numel(link.tendonActuation)<2, ...
                'elara:InvalidTendonActuationCount', ...
                "tendonActuation property must be scalar or empty.")

            % Check that inertia, mass, and center-of-mass transform arrays for
            % attached masses are either all empty or have the
            % correct dimensions
            assert( size(link.M_a, 3) == size(link.m_a, 2), ...
                'elara:InvalidAttachedBodyDimensions', ...
                ['Inconsistent array sizes for masses and inertia tensors of attached bodies. ' ...
                'To include attached bodies, both arrays must have values and the correct dimensions.'] ...
                );
            assert( size(link.M_a, 3) == size(link.g_a, 3), ...
                'elara:InvalidAttachedBodyDimensions', ...
                ['Inconsistent array sizes for transforms and inertia tensors of attached bodies. ' ...
                'To include attached bodies, both arrays must have values and the correct dimensions.'] ...
                );

            % Check properties of selection matrices
            r = size(link.Ba, 2); % Number of allowed deformation modes
            assert( size(link.Bc, 2) == 6-r, ...
                'elara:InvalidSelectionMatrix', ...
                "Selection matrix Bc has an incorrect number of columns." );
            assert( all(link.Ba == 0 | link.Ba == 1, "all") ...
                && all(link.Bc == 0 | link.Bc == 1, "all"), ...
                'elara:InvalidSelectionMatrix', ...
                'Ba and Bc must select columns of eye(6); signed or mixed deformation modes are not supported.');
            assert( all(link.Ba .' * link.Ba == eye(r), "all"), ...
                'elara:InvalidSelectionMatrix', ...
                "Selection matrix Ba must have orthonormal columns." );
            assert( all(link.Bc .' * link.Bc == eye(6-r), "all"), ...
                'elara:InvalidSelectionMatrix', ...
                "Selection matrix Bc must have orthonormal columns." );
            assert( all(link.Bc .' * link.Ba == zeros(6-r,r), "all"), ...
                'elara:InvalidSelectionMatrix', ...
                "Selection matrices Ba and Bc are not complementary." );

            Cgen = link.beamParameters.Cgen;
            assert( isequal(Cgen, diag(diag(Cgen))), ...
                'elara:UnsupportedBeamStiffness', ...
                'Cgen must be diagonal; coupled beam stiffness is not supported.');
        end
        function linkVis = getLinkVisualization(link, varargin)
            %% Initialize and return link visualization object
            linkVis = elara.visualization.FlexibleLinkVisualization(link, varargin{:});
        end
    end
end
