%% Build MATLAB documentation from Markdown files

docPath = fullfile(elara.internal.getToolboxRootFolder, "doc");

%% Clean old documentation
docdelete(docPath);

%% Convert to HTML
docconvert( ...
    fullfile(docPath, "*.md"), ...
    "Theme", "light", "Interpreter", "latex" ...
    );

% Note: If a 401 Unauthorized GitHub Error appears, it may be required to
% set a new GitHub personal access token via 
% setSecret("DocMaker GitHub token", Overwrite=true)

%% Include the MathJax boldsymbol extension
% This is required so LaTeX equations using \boldsymbol can be rendered
% correctly.
% This code downloads the correct extension specifically for MathJax
% 4.1.2; it may be required to adjust the version in the future.

extensionDir = fullfile(docPath, "resources", "input", "tex", "extensions");
if ~isfolder(extensionDir)
    mkdir(extensionDir);
end
websave(fullfile(extensionDir, "boldsymbol.js"), ...
    "https://cdn.jsdelivr.net/npm/mathjax@4.1.2/input/tex/extensions/boldsymbol.js");

%% Index documents
docindex(docPath);
