function object = convertMBSystemObject(MBSys, classname)
    %% Convert instances of the MBSystem and MBSystemSym class
    % between each other (small helper function)
    % See https://www.mathworks.com/matlabcentral/answers/350158-convert-a-struct-to-an-object-of-a-class
    arguments
        MBSys       (1,1) MBSystem
        classname   (1,1) string
    end

    % Only for MBSystem and MBSystemSym
    assert(classname == "MBSystem" || classname == "MBSystemSym");

    object = eval(classname);        %create object
    for fn = fieldnames(MBSys)'      %enumerate fields
        try
            object.(fn{1}) = MBSys.(fn{1});   %and copy
        catch
            warning('Could not copy field %s', fn{1});
        end
    end
end