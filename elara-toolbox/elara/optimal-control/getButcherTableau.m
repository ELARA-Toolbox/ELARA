function [A, b, c] = getButcherTableau(method)
    arguments
        method (1,1) string {mustBeMember(method,["RK2","RK4"])}
    end
    switch upper(method)
        case "RK2"  % Heun's method
            A = [0   0;
                1   0];
            b = [0.5; 0.5];
            c = [0; 1];

        case "RK4"  % Classic 4th order
            A = [0   0   0   0;
                0.5 0   0   0;
                0   0.5 0   0;
                0   0   1   0];
            b = [1; 2; 2; 1] / 6;
            c = [0; 0.5; 0.5; 1];

        otherwise
            error("Unknown method: %s", method);
    end
end