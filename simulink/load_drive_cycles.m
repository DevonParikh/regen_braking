% load_drive_cycles.m - load the VPython scenarios into MATLAB for Simulink.
% Run from the simulink/ folder after:  python simulink/export_for_simulink.py
%
% Creates:
%   params (veh, mot, bat, env, ctl, sc1, sc2, vp structs)  from params_regen.m
%   cycle_straight, cycle_corner   timeseries of reference speed (m/s)
%   path_corner                    [x z] midline points of the corner run (m)
%
% Powertrain Blockset "Drive Cycle Source" block: set Source = Workspace variable and
% enter cycle_straight (or cycle_corner). Units: m/s.

run('params_regen.m');

S = readtable('drive_cycle_straight.csv');
cycle_straight = timeseries(S.v_ref, S.t, 'Name', 'v_ref_straight');

C = readtable('drive_cycle_corner.csv');
cycle_corner = timeseries(C.v_ref, C.t, 'Name', 'v_ref_corner');
path_corner = [C.x, C.z];

fprintf('Loaded: straight %.1f s, corner %.1f s of reference speed.\n', S.t(end), C.t(end));
fprintf('VPython stored energy: straight %.1f kJ, corner %.1f kJ\n', ...
        vp.straight.E_stored_kJ, vp.corner.E_stored_kJ);
