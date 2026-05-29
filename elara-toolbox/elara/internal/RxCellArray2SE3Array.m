function A = RxCellArray2SE3Array(C)
    %% Cell array C = {R1, ..., RN, x1, ..., xN} to SE Array

    N = length(C)/2;
    C = reshape(C, N, 2);

    A = createArray(N, 1, "SE3");

    for iG = 1:N
        A(iG).R = C{iG,1};
        A(iG).x = C{iG,2};
    end
end