function CoAd = AdCayley(eta)
    %% Co-adjoint map for a Cayley-transformed se(3) element
    % Returns the co-adjoint representation of the SE(3) element obtained
    % from an se(3) element through the Cayley map. This avoids constructing
    % the full transformation matrix.
    arguments (Input)
        % (6,1) se(3) vector
        eta (6,1) double
    end
    arguments (Output)
        % Co-adjoint representation in (6,6) matrix form
        CoAd (6,6) double
    end
    [R,p] = elara.SE3.caySeparate( eta );

    CoAd = [
        R.',       -R.' * elara.SO3.skew(p)  ;
        zeros(3),   R.'
        ];
end
