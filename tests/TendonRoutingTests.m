% Regression tests for tendon-routing assembly and input matrices

addpath(fullfile(elara.internal.getToolboxRootFolder, "examples", "example-systems"));

%% Assemble a partially terminated discrete tendon

[link, nDisks, sDisks, gBackboneTendon] = makeDiscreteLink();

systemNum = elara.SystemNum(link);
iDisks = systemNum.iTendonDisksInLink(:,1);
assert(isequal(systemNum.nTendonDisksInLink(1), nDisks));
assert(isequal(systemNum.tendonIsActive(1,iDisks(1):iDisks(2)), ...
    [true, false, false, false]));
assert(isequal(systemNum.tendonIsActive(2,iDisks(1):iDisks(2)), ...
    [true, true, true, false]));

q = zeros(systemNum.nDoF, 1);
gRelNum = systemNum.computeJointTransformations(q);
BNum = systemNum.computeInputMatrix(q, gRelNum);
assert(isequal(size(BNum), [systemNum.nDoF, systemNum.nInputs]));

%% Build the same input matrix with CasADi MX coordinates

[link, ~, ~, ~] = makeDiscreteLink();
systemSym = elara.SystemSym(link);
qMX = casadi.MX.sym('q_tendon_routing', systemSym.nDoF, 1);
gRelMX = systemSym.computeJointTransformations(qMX);
BMX = systemSym.computeInputMatrix(qMX, gRelMX);
nonemptyBlocks = BMX(~cellfun(@isempty, BMX));
assert(~isempty(nonemptyBlocks));
assert(isa(nonemptyBlocks{1}, 'casadi.MX'));
casadi.Function('tendon_input_matrix_mx', {qMX}, {nonemptyBlocks{1}});

%% Preserve routing metadata during numeric-to-symbolic conversion

[link, ~, ~, ~] = makeDiscreteLink();
systemNum = elara.SystemNum(link);
convertedSystemSym = elara.internal.systemNum2SystemSym(systemNum);
assert(isequal(convertedSystemSym.iTendonDisksInLink, ...
    systemNum.iTendonDisksInLink));
assert(isequal(convertedSystemSym.iTendonSectionsBetweenDisks, ...
    systemNum.iTendonSectionsBetweenDisks));
assert(isequal(convertedSystemSym.tendonSectionFrameIndex, ...
    systemNum.tendonSectionFrameIndex));

%% Allocate routing transforms for every disk across multiple links

[link1, nDisks, sDisks, gBackboneTendon] = makeDiscreteLink();
link2 = link1;
link2.parentLink = 1;
link2.isCantilever = false;
link2.tendonActuation = elara.DiscreteTendonActuation( ...
    nDisks * ones(2,1), sDisks, gBackboneTendon(:,:,1:2,:));
multiLinkSystem = elara.SystemNum([link1; link2]);
assert(size(multiLinkSystem.gBackboneTendon, 4) == 2*nDisks);
assert(isequal(multiLinkSystem.gBackboneTendon(:,:,3,nDisks+1), eye(4)));
assert(size(multiLinkSystem.tendonIsActive, 2) == 2*nDisks);

qMulti = zeros(multiLinkSystem.nDoF, 1);
gRelMulti = multiLinkSystem.computeJointTransformations(qMulti);
BMulti = multiLinkSystem.computeInputMatrix(qMulti, gRelMulti);
assert(isequal(size(BMulti), ...
    [multiLinkSystem.nDoF, multiLinkSystem.nInputs]));

%% Map continuous routing sections to beam frames after rigid links

links = systemDef_rigid_flexible_robot("nSegments", uint8(3));
sDisksContinuous = linspace(0, links(end).L, ...
    double(links(end).nSegments) + 1).';
xTendon = repmat({@(s)[0.005; 0; 0]}, 4, 1);
dxTendon = repmat({@(s)[0; 0; 0]}, 4, 1);
ddxTendon = repmat({@(s)[0; 0; 0]}, 4, 1);
links(end).tendonActuation = elara.ContinuousTendonActuation( ...
    links(end).L * ones(4,1), sDisksContinuous, ...
    xTendon, dxTendon, ddxTendon);
rigidFlexibleSystem = elara.SystemNum(links);
qRigidFlexible = zeros(rigidFlexibleSystem.nDoF, 1);
gRelRigidFlexible = rigidFlexibleSystem.computeJointTransformations( ...
    qRigidFlexible);
BRigidFlexible = rigidFlexibleSystem.computeInputMatrix( ...
    qRigidFlexible, gRelRigidFlexible);
assert(isequal(size(BRigidFlexible), ...
    [rigidFlexibleSystem.nDoF, rigidFlexibleSystem.nInputs]));

%% Keep following-link section offsets correct after an early final disk

[link1, ~, ~, gBackboneTendon] = makeDiscreteLink();
tailDisks = [0; 0.2*link1.L; 0.6*link1.L];
link1.tendonActuation = elara.DiscreteTendonActuation( ...
    3 * ones(4,1), tailDisks, gBackboneTendon(:,:,:,1:3));
[link2, nDisks, sDisks, gBackboneTendon] = makeDiscreteLink();
link2.parentLink = 1;
link2.isCantilever = false;
tailSystem = elara.SystemNum([link1; link2]);
qTail = zeros(tailSystem.nDoF, 1);
gRelTail = tailSystem.computeJointTransformations(qTail);
BTail = tailSystem.computeInputMatrix(qTail, gRelTail);
assert(isequal(size(BTail), [tailSystem.nDoF, tailSystem.nInputs]));

%% Reject invalid discrete-routing indices before assembly

[~, nDisks, sDisks, gBackboneTendon] = makeDiscreteLink();
didRejectFractionalTermination = false;
try
    elara.DiscreteTendonActuation([1.5; nDisks], sDisks, gBackboneTendon);
catch
    didRejectFractionalTermination = true;
end
assert(didRejectFractionalTermination);

function [link, nDisks, sDisks, gBackboneTendon] = makeDiscreteLink()
    link = systemDef_continuum_manipulator("nSegments", 3);
    link.hasTCP = false;
    nDisks = link.nSegments + 1;
    sDisks = linspace(0, link.L, nDisks).';
    gBackboneTendon = link.tendonActuation.gBackboneTendon;
    terminationDisks = [2; nDisks; nDisks; nDisks];
    link.tendonActuation = elara.DiscreteTendonActuation( ...
        terminationDisks, sDisks, gBackboneTendon);
end
