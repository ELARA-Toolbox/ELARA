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
            assert(size(gBackboneTendons, 4) == length(sDisks))
            assert(size(gBackboneTendons, 3) == length(terminationDisks));
            assert(length(sDisks) >= max(terminationDisks));

            obj.gBackboneTendon = gBackboneTendons;
            obj.sDisks = sDisks;
            obj.terminationDisks = terminationDisks;
        end
    end
end