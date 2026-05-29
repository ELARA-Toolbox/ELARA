classdef SE3
    %% Class Representing an Element of SE3

    properties
        R (3,3) = eye(3);
        x (3,1) = zeros(3,1);
    end

    methods
        function obj = SE3(R,x)
            % Note: We only use a very "lightweight" constructor for
            % efficiency
            if nargin == 2
                obj.R = R;
                obj.x = x;
            end
            % arguments
            %     R (3,3) = eye(3);
            %     x (3,1) = zeros(3,1);
            % end
            % obj.R = R;
            % obj.x = x;
        end
        function gMat = mat(g)
            gMat = [g.R, g.x; 0,0,0,1];
        end
        function gInv = inv(g)
            gInv = SE3(g.R.', -g.R.'*g.x);
        end
        function A = Ad(g)
            % Ad operator in matrix form
            zrs = getZeros(g.R);
            A = [
                g.R,              zrs(3,3);
                skewSO3(g.x)*g.R, g.R;
                ];
        end
        function A = AdInv(g)
            % Inverse Ad operator in matrix form
            zrs = getZeros(g.R);
            A = [
                g.R.',               zrs(3,3);
                -g.R.'*skewSO3(g.x), g.R.'
                ];
        end
        function xi = cayInv(g)
            % Inverse cayley transform
            eyeF = getEye(g.R);
            omegaH = 2 / (1 + trace(g.R) ) * (g.R - g.R.');
            omega = [ omegaH(3,2); omegaH(1,3); omegaH(2,1) ];
            v  = 2 * ( (g.R + eyeF(3)) \ g.x );
            xi = [ omega; v];
        end
    end

    %% Overloaded Operators
    methods
        function g = mtimes(g1, g2)
            %% Matrix Product as Product on SE3
            g = SE3(g1.R * g2.R, g1.x + g1.R*g2.x);
        end
        function g = mrdivide(g1, g2)
            %% Matrix Right Division as Operation on SE3
            g = SE3(g1.R * g2.R.', g1.x - g1.R*g2.R.'*g2.x);
        end
        function g = mldivide(g1, g2)
            %% Matrix Left Division as Operation on SE3
            g = SE3(g1.R.' * g2.R, -g1.R.' * g1.x + g1.R.'*g2.x);
        end

    end
end