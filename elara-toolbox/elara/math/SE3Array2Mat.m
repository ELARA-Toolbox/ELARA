function gNum = SE3Array2Mat(gSE3)
    %% Convert array of SE3 objects to double array of 4x4 matrices
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments (Input)
        gSE3  elara.SE3.Element
    end
    arguments (Output)
        gNum  double {mustBeSE3MatrixArray}
    end
    gNum = zeros(4,4,numel(gSE3));
    for iG = 1:numel(gSE3)
        gNum(:,:,iG) = gSE3(iG).mat;
    end
    gNum = reshape(gNum, [4,4,size(gSE3)]);
end

