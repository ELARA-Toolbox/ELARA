%% Run Unit Tests for Cayley Maps
%
% Test structure after 
% https://de.mathworks.com/help/matlab/matlab_prog/write-script-based-unit-tests.html
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

result = runtests('CayleyMapTests');
disp( table(result) ); 