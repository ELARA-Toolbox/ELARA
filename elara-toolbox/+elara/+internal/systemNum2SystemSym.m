function systemSym = systemNum2SystemSym(systemNum)
    %% Convert an elara.SystemNum instance to elara.SystemSym
    % between each other (small helper function)
    % See https://www.mathworks.com/matlabcentral/answers/350158-convert-a-struct-to-an-object-of-a-class
    arguments
        systemNum       (1,1) elara.SystemNum
    end

    systemSym = elara.SystemSym;        %create object
    for fn = fieldnames(systemNum)'      %enumerate fields
        try
            systemSym.(fn{1}) = systemNum.(fn{1});   %and copy
        catch
            %warning('Could not copy field %s', fn{1});
        end
    end
    for fn = fieldnames(systemNum.frames)'      %enumerate fields
        try
            systemSym.frames.(fn{1}) = systemNum.frames.(fn{1});   %and copy
        catch
            %warning('Could not copy field %s', fn{1});
        end
    end
    systemSym.frames.g_cm = elara.SE3.matrix2Element(systemNum.frames.g_cm);
    systemSym.frames.MGen = squeeze(num2cell(systemNum.frames.MGen,[1,2]));


end
