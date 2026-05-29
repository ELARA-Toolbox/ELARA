function CoAd = ColAdCayley(eta)
    %% Co-Ad Map for a Cayley-transformed se3 element
    % Helper function which returns the Co-Ad representation for the SE(3)
    % element, which has been calculated from an se(3) element via the
    % cayley map
    % This is used for faster computations / reduce overhead.
    arguments (Input)
        % (6,1) se3 vector
        eta (6,1) double
    end
    arguments (Output)
        % Ad representation in (6,6) matrix form
        CoAd (6,6) double
    end
    [R,p] = caySE3sep( eta );

    CoAd = [
        R.',       -R.' * skewSO3(p)  ;
        zeros(3),   R.'
        ];
end
