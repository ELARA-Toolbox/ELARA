function gSE3 = matrix2Element(gNum)
    %% Convert an array of 4x4 matrices to an array of SE(3) elements
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments (Input)
        gNum  double {elara.internal.validation.mustBeSE3Matrix}
    end
    arguments (Output)
        gSE3  elara.SE3.Element
    end
    dims = size(gNum);
    dims = dims(3:end);
    if isempty(dims)
        gSE3 = elara.SE3.Element(gNum(1:3,1:3), gNum(1:3,4));
        return;
    end
    if isscalar(dims)
        % Add a singleton dimension if the input is an SE(3) vector
        % so that createArray outputs correct dimensions
        dims = [dims, 1];
    end
    n = numel(gNum)/16;
    gSE3 = createArray(dims,"elara.SE3.Element");
    gNum1D = reshape(gNum, 4, 4, []);
    for iG = 1:n
        gSE3(iG).R = gNum1D(1:3,1:3,iG);
        gSE3(iG).x = gNum1D(1:3,4,iG);
    end
end

