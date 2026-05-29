function Lambda = caySO3( om )
    %% Cayley map for SO(3)
    % Implements the Cayley map for SO(3): cay : so(3) -> SO(3)
    %
    % Source: [Dem+14, p.9], eq. 14
    %
    % Input om:   so(3) element in *vector* form (omega)
    % Output Lambda: Corresponding rotation matrix
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        om (3,1)
    end

    omH = skewSO3(om);

    Lambda = eye(3) + 4 / (4 + om.'*om) * (omH + ( omH*omH / 2 ));
end

