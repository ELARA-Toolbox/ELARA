classdef ContinuousTendonActuation < elara.abstract.TendonActuation
    properties (Constant)
        tendonActuationType = 'continuous';
    end
    methods
        function obj = ContinuousTendonActuation(LTermination, sDisks, x_td_funs,x_td_ds_funs, x_td_dds_funs)
            arguments
                % Vector of lengths, at which the cables terminate (nCables,1)
                LTermination (:,1) double
                
                % s Position of Disks (zero position not included
                sDisks          (:,1) double 

                % Cell array of function handles that define the cable path (cable
                % position relative to the backbone/centerline) over beam length
                % dimensions (nCables, 1)
                % The functions must have the form
                %           [x;y;z] = f(s)
                % where f : R -> R^3 returns the cable path at s
                x_td_funs        (:,1) cell

                % First and second derivatives of the cable path functions w.r.t. s
                x_td_ds_funs     (:,1) cell = {}
                x_td_dds_funs    (:,1) cell = {}
      
            end
            if isempty(x_td_ds_funs) 
                s = sym("s");
                x_td_ds_funs = cellfun( ...
                @(x) matlabFunction( diff(x(s), s, 1), "Vars", s ), ...
                x_td_funs, ...
                'UniformOutput', false);
            end
            if isempty(x_td_dds_funs)
                s = sym("s");
                x_td_dds_funs = cellfun( ...
                    @(x) matlabFunction( diff(x(s), s, 2), "Vars", s ), ...
                    x_td_funs, ...
                    'UniformOutput', false);
            end
             % Make sure function array sizes match
            assert(length(x_td_funs) == length(x_td_ds_funs));
            assert(length(x_td_funs) == length(x_td_dds_funs));
            assert(length(x_td_funs) == length(LTermination));
            
            obj.sDisks = sDisks;

            nDisks  = length(sDisks);
            nTendons = length(x_td_funs);

            % Compute relative transformation / position of the cable path
            % at the nodes
            obj.gBackboneTendon = repmat(eye(4), [1,1,nTendons,nDisks]);
           % x_cm = zeros(3, nNodes, nCables);
            
            obj.terminationDisks = zeros(nTendons, 1);
            
            for iTendon = 1:nTendons
                % Tendon positions
                xBackboneTendons = cell2mat( ...
                    arrayfun( x_td_funs{iTendon}, sDisks , ...
                    'UniformOutput', false).' ...
                    );

                %%% Tangential cable path rotation matrices (Frenet-Serret
                %%% frames)
                R = zeros(3, 3, nDisks);

                % Get function handles
                f_ds  = x_td_ds_funs{iTendon}; 
                f_dds = x_td_dds_funs{iTendon}; 

                % Compute basis vectors of the Frenet-Serret frame
                for iDisk = 1:nDisks
                    sDisk = sDisks(iDisk);
                    % Tangent vector t
                    t = f_ds(sDisk);% / norm(f_ds(lNodes(iN)));
                    t(3) = 1;
                    t = t / norm(t);

                    % Normal vector n
                    if any(f_dds(sDisk))
                        n = f_dds(sDisk) / norm(f_dds(sDisk));
                    else
                        n = [1;0;0];
                    end
                    % Bi-normal vector b
                    b = cross(n, t);

                    % Store in rotation matrix; z-axis is aligned with
                    % tangent vector
                    R(:,:,iDisk) = [b,n,t];
                end

                % Cable mount transformations
                obj.gBackboneTendon(1:3,4, iTendon, :) = xBackboneTendons;
                obj.gBackboneTendon(1:3,1:3, iTendon, :) = R;
                
                % Compute Disk at which each tendon ends
                obj.terminationDisks(iTendon) = find(sDisks >= LTermination(iTendon) , 1);
            end

        end
    end
end