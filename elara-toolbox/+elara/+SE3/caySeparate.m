function [R,x] = caySeparate( xi )
    %% Cayley map for SE(3) (with separate R,x outputs)
    % Implements the Cayley map for SE(3): cay : se(3) -> SE(3)
    %
    % Source: [Dem+14, p.10], eq. 19
    % Follows the convention for se(3) elements in vector form: [omega; v]
    %
    % Input xi: se(3) element in *vector* form
    % Outputs R and p: Corresponding element of SE(3)
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

    omH = elara.SO3.skew(om);
    
    R = elara.SO3.cay( om );
    x = ( 4 / (4 + om.'*om) ) * ( eye(3) + 1/2 * omH + 1/4 * (om*om.') ) * v;
end

