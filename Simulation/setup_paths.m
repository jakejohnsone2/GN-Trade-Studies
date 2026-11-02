function setup_paths()
%SETUP_PATHS Add every Simulation folder to the MATLAB path.
addpath(genpath(fileparts(mfilename('fullpath'))));
end
