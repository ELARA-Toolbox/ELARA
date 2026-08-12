function rootPath = getToolboxRootFolder
    %% Get the absolute root folder of the toolbox
    rootPath = fileparts(fileparts(fileparts(mfilename("fullpath"))));
end