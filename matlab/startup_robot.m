% Run this file with
%   run('/path/to/git/repo/matlab_config/startup_robot.m');

matlab_dir = fileparts( mfilename('fullpath') );

addpath(matlab_dir);

robotpath = fullfile(matlab_dir, 'rvctools');
vlfeatpath = fullfile(matlab_dir, 'vlfeat-0.9.20');

% Download the RTB. 
if ~exist(robotpath, 'dir') & ~exist('rtbdemo')
    dodownload = input('RVC toolbox not found. Do you want to download it? [Y/n] ', 's');
    if isempty(dodownload) | dodownload ~= 'n'
        files = { 'contrib.zip',  'contrib2.zip',  'vision-3.4.zip', 'robot-9.10.zip' };
        for i = 1:4
            fprintf('Downloading %s...\n', files{i});
            unzip(sprintf('http://renaud-detry.net/teaching/info0948/data/%s', files{i}));
            fprintf('Downloading %s: done.\n', files{i});
        end
    end
end

% Download an updated VLFeat (RTB comes with an older version). 
if ~exist(vlfeatpath, 'dir')
    dodownload = input('Updated VLFeat not found. Do you want to download it? (Supplementary vision functions, useful but not required.) [Y/n] ', 's');
    if isempty(dodownload) | dodownload ~= 'n'
        fprintf('Downloading updated VLFeat...\n');
        unzip('http://www.montefiore.ulg.ac.be/~tcuvelier/vlfeat-0.9.20.zip');
        fprintf('Downloading updated VLFeat: done.\n');
        fprintf('If you have troubles using VLFeat, try recompiling the MEX files (vl_compile).\n');
    end
end

% Add the toolboxes to the path. 
if exist(robotpath, 'dir')
    run(fullfile(robotpath, 'startup_rvc.m'));
end
if exist(vlfeatpath, 'dir')
    run(fullfile(vlfeatpath, 'toolbox', 'vl_setup.m'));
    fprintf('Run vl_demo to explore VLFeat\n');
end
