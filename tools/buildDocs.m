%% Build MATLAB documentation from Markdown files

docPath = fullfile(elara.internal.getToolboxRootFolder, "doc");
extensionSource = fullfile(docPath, "third-party", "mathjax", "boldsymbol.js");

assert(isfile(extensionSource), "elara:buildDocs:MissingMathJaxExtension", ...
    "Missing MathJax extension: %s", extensionSource);

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
% Keep the source under doc/third-party, since docdelete removes doc/resources.
% Copy it to MathJax's default extension path for viewing the shipped HTML.
% This extension matches MathJax 4.1.2 bundled with DocMaker 0.8.0. Update
% the extension and its license if DocMaker's bundled MathJax changes.

extensionDir = fullfile(docPath, "resources", "input", "tex", "extensions");
if ~isfolder(extensionDir)
    mkdir(extensionDir);
end
copyfile(extensionSource, fullfile(extensionDir, "boldsymbol.js"));

%% Index documents
docindex(docPath);
