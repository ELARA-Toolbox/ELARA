%% Build MATLAB documentation from Markdown files

docPath = fullfile(elara.internal.getToolboxRootFolder, "doc");

%% Clean old documentation
docdelete(docPath);

%% Convert to HTML
docconvert(fullfile(docPath, "*.md"), "Theme", "light");

% Note: If a 401 Unauthorized GitHub Error appears, it may be required to
% set a new GitHub personal access token via 
% setSecret("DocMaker GitHub token", Overwrite=true)

%% Index documents
docindex(docPath);
