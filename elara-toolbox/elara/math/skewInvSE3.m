function V = skewInvSE3(gSk)
    %% skewInvSE3
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % Department of Mechanical Engineering
    % Technical University of Munich
    % 03/2021
    %
    % Inverse hat map for an element of se(3)
    % (i.e. skew-symmetric form of a 6x1 vector [v; omega].
    % For more details, see the skewInv function (for so(3)).
    
    arguments
        gSk (4,4)
    end
    
    % Extract skew-symmetric matrix (so(3)) and position vector
    R = gSk(1:3, 1:3);
    v = gSk(1:3, 4);
    
    % Give 6x1 vector
    V = [skewInv(R); v];
end