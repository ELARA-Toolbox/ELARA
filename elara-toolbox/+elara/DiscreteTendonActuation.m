classdef DiscreteTendonActuation < elara.abstract.TendonActuation
    properties (Constant)
        tendonActuationType = 'discrete';
    end
    methods
        function obj = DiscreteTendonActuation(terminationDisks,sDisks, gBackboneTendons)
            arguments
                terminationDisks     (:,1) double % nTendons x 1         
                sDisks              (:,1) double {mustBeNonnegative} % nDisks x 1
                gBackboneTendons    (4,4,:,:) double % 4 x 4 x nTendons x nDisks
            end
            nDisks = length(sDisks);
            assert(nDisks >= 2, ...
                'Discrete tendon routing requires at least two disks.');
            assert(all(isfinite(sDisks)) && all(diff(sDisks) > 0), ...
                'Disk positions must be finite and strictly increasing.');
            assert(size(gBackboneTendons, 4) == nDisks, ...
                'The routing-transform array must contain one entry per disk.');
            assert(size(gBackboneTendons, 3) == length(terminationDisks), ...
                'The routing-transform array must contain one entry per tendon.');
            assert(all(isfinite(terminationDisks)) && ...
                all(terminationDisks == floor(terminationDisks)) && ...
                all(terminationDisks >= 2) && all(terminationDisks <= nDisks), ...
                'Termination disks must be integer indices from 2 through nDisks.');

            obj.gBackboneTendon = gBackboneTendons;
            obj.sDisks = sDisks;
            obj.terminationDisks = terminationDisks;
        end
    end
end
