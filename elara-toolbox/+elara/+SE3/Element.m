classdef Element
    %% Class representing an element of SE(3)

    properties
        R (3,3) = eye(3);
        x (3,1) = zeros(3,1);
    end

    methods
        function obj = Element(R,x)
            % Keep construction lightweight because this value class is
            % instantiated frequently in symbolic calculations.
            if nargin == 2
                obj.R = R;
                obj.x = x;
            end
        end
        function gMat = mat(g)
            gMat = [g.R, g.x; 0,0,0,1];
        end
        function gInv = inv(g)
            gInv = elara.SE3.Element(g.R.', -g.R.'*g.x);
        end
        function A = Ad(g)
            % Ad operator in matrix form
            zrs = elara.internal.math.getZeros(g.R);
            A = [
                g.R,              zrs(3,3);
                elara.SO3.skew(g.x)*g.R, g.R;
                ];
        end
        function A = AdInv(g)
            % Inverse Ad operator in matrix form
            zrs = elara.internal.math.getZeros(g.R);
            A = [
                g.R.',               zrs(3,3);
                -g.R.'*elara.SO3.skew(g.x), g.R.'
                ];
        end
        function xi = cayInv(g)
            % Inverse cayley transform
            eyeF = elara.internal.math.getEye(g.R);
            omegaH = 2 / (1 + trace(g.R) ) * (g.R - g.R.');
            omega = [ omegaH(3,2); omegaH(1,3); omegaH(2,1) ];
            v  = 2 * ( (g.R + eyeF(3)) \ g.x );
            xi = [ omega; v];
        end
    end

    %% Overloaded Operators
    methods
        function g = mtimes(g1, g2)
            %% Matrix product as a product on SE(3)
            g = elara.SE3.Element(g1.R * g2.R, g1.x + g1.R*g2.x);
        end
        function g = mrdivide(g1, g2)
            %% Matrix right division as an operation on SE(3)
            g = elara.SE3.Element(g1.R * g2.R.', g1.x - g1.R*g2.R.'*g2.x);
        end
        function g = mldivide(g1, g2)
            %% Matrix left division as an operation on SE(3)
            g = elara.SE3.Element(g1.R.' * g2.R, -g1.R.' * g1.x + g1.R.'*g2.x);
        end

    end
end
