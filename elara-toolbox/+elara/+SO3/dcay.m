function T = dcay(omega)
    %% Right-trivialized derivative of the Cayley map for SO(3)
    arguments
        omega (3,1)
    end

    omegaHat = elara.SO3.skew(omega);
    T = 2 / (4 + omega.'*omega) * (2*eye(3) + omegaHat);
end
