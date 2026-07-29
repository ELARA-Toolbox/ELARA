function T = dcayInv(omega)
    %% Inverse right-trivialized derivative of the Cayley map for SO(3)
    arguments
        omega (3,1)
    end

    omegaHat = elara.SO3.skew(omega);
    T = eye(3) - 1/2*omegaHat + 1/4*(omega*omega.');
end
