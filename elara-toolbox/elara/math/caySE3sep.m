function [R,x] = caySE3sep( xi )
    %% Cayley map for SE(3) (with separate R,x outputs)
    % Implements the Cayley map for SE(3): cay : se(3) -> SE(3)
    %
    % Source: [Dem+14, p.10], eq. 19
    % Follows convention for se3 elements in vector form: [omega; v]
    %
    % Input xi: se(3) element in *vector* form
    % Output g: Corresponding element of SE3 in matrix form
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        xi (6,1)
    end

    om = xi(1:3);
    v  = xi(4:6);

    omH = skew(om);
    
    R = caySO3( om );
    x = ( 4 / (4 + om.'*om) ) * ( eye(3) + 1/2 * omH + 1/4 * (om*om.') ) * v;
end

