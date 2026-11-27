%% Run Unit Tests
%
% Test structure after 
% https://de.mathworks.com/help/matlab/matlab_prog/write-script-based-unit-tests.html
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

%% Cayley Map Tests
result = runtests('CayleyMapTests');
disp( table(result) );

%% Exponential Map Tests
result = runtests('ExpMapTests');
disp( table(result) );

%% Numeric/Symbolic System Function Tests
result = runtests('SystemFunctionTests');
disp( table(result) );

%% Inverse Dynamics Tests
result = runtests('InverseDynamicsTests');
disp( table(result) );

%% Tendon Routing Regression Tests
result = runtests('TendonRoutingTests');
disp( table(result) );

%% Simulation Regression Tests
result = runtests('SimulationRegressionTests');
disp( table(result) );

%% Shared Diagnostic Identifier Tests
result = runtests('DiagnosticIdentifierTests');
disp( table(result) );
