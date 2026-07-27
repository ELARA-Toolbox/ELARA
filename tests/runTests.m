%% Run Unit Tests
%
% Test structure after 
% https://de.mathworks.com/help/matlab/matlab_prog/write-script-based-unit-tests.html
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

%% Run Cayley Map Tests
result = runtests('CayleyMapTests');
disp( table(result) ); 

%% Run Exponential Map Tests
result = runtests('ExpMapTests');
disp( table(result) ); 

%% Run Numeric/Symbolic System Function Tests
result = runtests('SystemFunctionTests');
disp( table(result) ); 