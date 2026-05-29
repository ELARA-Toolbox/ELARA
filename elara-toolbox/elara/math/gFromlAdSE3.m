function g = gFromlAdSE3(Ad)
    % Extract SE3 element from its Ad representation (6x6 matrix)
    % convention for se3 elements in vector form: [omega; v]
    % g is the SE3 element in (4x4) matrix representation
    arguments
        Ad (6,6)
    end

    R  = Ad(1:3,   1:3);
    PR = Ad(4:end, 1:3);

    % Compute p
    P = PR * R';

    % Compute g
    g = [
        R,          skewInv(P);
        zeros(1,3), 1
        ];
end