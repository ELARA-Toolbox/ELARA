function gSE3 = SE3MatArray2SE3Array(gNum)
    %% Convert array of 4x4 SE3 Matrices to array of SE3 objects
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments (Input)
        gNum  double {mustBeSE3MatrixArray}
    end
    arguments (Output)
        gSE3  elara.SE3.Element
    end
    dims = size(gNum);
    dims = dims(3:end);
    if isscalar(dims)
        % Add singleton dimension if the input array is just a SE3 vector
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

