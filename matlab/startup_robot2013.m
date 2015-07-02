% Run this file with
%   run('/path/to/git/repo/matlab_config/startup_robot.m');

matlab_dir = fileparts( mfilename('fullpath') );

addpath(matlab_dir);

robotpath = fullfile(matlab_dir, 'rvctools');

if ~exist(robotpath,'dir') & ~exist('rtbdemo'),
  dodownload = input('RVC toolbox not found. Do you want to download it? [Y/n] ', 's');
  if isempty(dodownload) | dodownload ~= 'n',
    files = { 'contrib.zip',  'contrib2.zip',  'vision-3.3.zip', 'robot-9.8.zip' };
    for i = 1:4,
      fprintf('Downloading %s...\n', files{i});
      unzip(sprintf('http://renaud-detry.net/teaching/info0948/data/%s', files{i}));
      fprintf('Downloading %s: done.\n', files{i});
    end
  end
end

if exist(robotpath,'dir')
    run(fullfile(robotpath, 'startup_rvc.m'));
end
