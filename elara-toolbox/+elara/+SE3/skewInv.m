function V = skewInv(gSk)
    %% elara.SE3.skewInv
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    % 03/2021
    %
    % Inverse hat map for an element of se(3)
    % (i.e., the matrix form of a 6x1 vector [omega; v]).
    % For more details, see elara.SO3.skewInv.
    
    arguments
        gSk (4,4)
    end
    
    % Extract skew-symmetric matrix (so(3)) and position vector
    R = gSk(1:3, 1:3);
    v = gSk(1:3, 4);
    
    % Return the 6x1 vector [omega; v]
    V = [elara.SO3.skewInv(R); v];
end
