function setup_project(part)
% Select one deliverable without putting conflicting controller classes on path.
% Add external toolbox folders to MATLAB's path before calling this function.
if nargin < 1, part = 'nonlinear-tracking'; end
here = fileparts(mfilename('fullpath'));
deps = fullfile(fileparts(here),'.dependencies');
if isfolder(fullfile(deps,'casadi')), addpath(fullfile(deps,'casadi')); end
if isfolder(fullfile(deps,'yalmip')), addpath(genpath(fullfile(deps,'yalmip'))); end
if isfolder(fullfile(deps,'mpt3','source','mpt'))
    addpath(genpath(fullfile(deps,'mpt3','source','mpt')));
end
names = {'dynamics','linearization','tracking','offset-free','tube-mpc', ...
         'nonlinear-tracking','nonlinear-avoidance'};
folders = {'01-dynamics','02-linearization','03-tracking','04-offset-free', ...
           '05-tube-mpc','06-nonlinear-tracking','07-nonlinear-avoidance'};
idx = find(strcmp(part,names),1);
assert(~isempty(idx),'Setup:Part','Unknown part: %s',part);
% Remove only this repository's competing class folders.
for i = 1:numel(folders)
    p = fullfile(here,'deliverables',folders{i});
    if contains([path pathsep],[p pathsep]), rmpath(p); end
end
optional = fullfile(fileparts(here),'portfolio_demo','src');
if contains([path pathsep],[optional pathsep]), rmpath(optional); end
addpath(fullfile(here,'common'));
addpath(fullfile(here,'deliverables',folders{idx}));
if ~strcmp(part,'dynamics')
    assert(exist('casadiMEX','file') == 3,'Setup:CasADi', ...
        'Add the official CasADi MATLAB distribution to your path. See README.md.');
end
if any(strcmp(part,{'tracking','offset-free','tube-mpc'}))
    assert(exist('sdpvar','file') == 2,'Setup:YALMIP','Add YALMIP to your path.');
    assert(exist('Polyhedron','class') == 8,'Setup:MPT','Add and initialize MPT3.');
end
end
