function MBSysSym = MBSystemNum2MBSystemSym(MBSysNum)
    %% Convert instances of the MBSystem and MBSystemSym class
    % between each other (small helper function)
    % See https://www.mathworks.com/matlabcentral/answers/350158-convert-a-struct-to-an-object-of-a-class
    arguments
        MBSysNum       (1,1) MBSystemNum
    end

    MBSysSym = MBSystemSym;        %create object
    for fn = fieldnames(MBSysNum)'      %enumerate fields
        try
            MBSysSym.(fn{1}) = MBSysNum.(fn{1});   %and copy
        catch
            %warning('Could not copy field %s', fn{1});
        end
    end
    for fn = fieldnames(MBSysNum.frameData)'      %enumerate fields
        try
            MBSysSym.frameData.(fn{1}) = MBSysNum.frameData.(fn{1});   %and copy
        catch
            %warning('Could not copy field %s', fn{1});
        end
    end
    MBSysSym.frameData.g_cm = SE3MatArray2SE3Array(MBSysNum.frameData.g_cm);
    MBSysSym.frameData.MGen = squeeze(num2cell(MBSysNum.frameData.MGen,[1,2]));


end