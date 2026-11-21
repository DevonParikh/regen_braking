function export_results(filename, t, speed, varargin)
% export_results  Write Simulink results to the CSV format compare_simulink.py reads.
%
%   export_results('results_straight.csv', t, speed, 'F_regen', Fr, 'E_stored', E, ...)
%
% t, speed are required (s, m/s). Optional name/value pairs (SI units, same length as t):
%   F_regen (N), F_fric (N), E_stored (J), SOC (0-1), N_f (N), N_r (N), F_lat (N)
%
% Example with "To Workspace" blocks saving timeseries (out = sim('your_model')):
%   ts = out.speed;
%   export_results('results_straight.csv', ts.Time, ts.Data, ...
%       'F_regen', out.F_regen.Data, 'E_stored', out.E_stored.Data);
% Signals from the Simulation Data Inspector / logsout:
%   s = out.logsout.get('speed').Values;   % then use s.Time, s.Data

t = t(:); speed = speed(:);
T = table(t, speed);
for k = 1:2:numel(varargin)
    name = varargin{k};
    val = varargin{k + 1};
    val = val(:);
    if numel(val) ~= numel(t)
        error('export_results:length', '%s has %d samples but t has %d', name, numel(val), numel(t));
    end
    T.(name) = val;
end
writetable(T, filename);
fprintf('Wrote %s (%d rows, columns: %s)\n', filename, height(T), strjoin(T.Properties.VariableNames, ', '));
end
