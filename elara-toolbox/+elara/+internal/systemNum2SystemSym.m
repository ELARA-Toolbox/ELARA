function MBSysSym = systemNum2SystemSym(MBSysNum)
    %% Convert an elara.SystemNum instance to elara.SystemSym
    % between each other (small helper function)
    % See https://www.mathworks.com/matlabcentral/answers/350158-convert-a-struct-to-an-object-of-a-class
    arguments
        MBSysNum       (1,1) elara.SystemNum
    end

    MBSysSym = elara.SystemSym;        %create object
    for fn = fieldnames(MBSysNum)'      %enumerate fields
        try
            MBSysSym.(fn{1}) = MBSysNum.(fn{1});   %and copy
        catch
            %warning('Could not copy field %s', fn{1});
        end
    end
    for fn = fieldnames(MBSysNum.frames)'      %enumerate fields
        try
            MBSysSym.frames.(fn{1}) = MBSysNum.frames.(fn{1});   %and copy
        catch
            %warning('Could not copy field %s', fn{1});
        end
    end
    MBSysSym.frames.g_cm = SE3MatArray2SE3Array(MBSysNum.frames.g_cm);
    MBSysSym.frames.MGen = squeeze(num2cell(MBSysNum.frames.MGen,[1,2]));


end
