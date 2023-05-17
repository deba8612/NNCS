function [inputs1_cell, output1_cell] = generateLaneKeepingTrace()
    global Ts T
    global v0 y0 seed
    
    % Set seed for random number generator
    seed = randi([0, 1000]);

    % Define the sample time (Ts) and simulation duration (T) in seconds
    Ts = 0.1;
    T = 1000;

    function md = getCurvature(Vx,time)
% Get previewed curvature from desired X and Y positions for LKA
%
% Inputs:
%   Vx: longitudinal velocity
%   time: time vector
%
% Outputs:
%   md: previewed curvature

% Desired X position
Xref = Vx*time;

% Desired Y position
z1 = (2.4/50)*(Xref-27.19)-1.2;
z2 = (2.4/43.9)*(Xref-56.46)-1.2;
Yref = 8.1/2*(1+tanh(z1)) - 11.4/2*(1+tanh(z2));

% Desired curvature
DX = gradient(Xref,0.1);
DY = gradient(Yref,0.1);
D2Y = gradient(DY,0.1);
curvature = DX.*D2Y./(DX.^2+DY.^2).^(3/2);

% Stored curvature (as input for LKA)
md.time = time;
md.signals.values = curvature';
md.DataType = 'double';

    end

    % Specify the initial velocity and lateral position
    v0 = 25 + 1.5 - 3*rand(1);   % initial velocity (m/s)
    y0 = 0 + 1.5 - 3*rand(1);    % initial lateral position (m)

    % Run the simulation
    sim('mpcLKAsystem')

    % Extract input and output data for training
    v_ego = logsout.get('v_ego').Values.Data;         % input: ego vehicle velocity (m/s)
    y_ego = logsout.get('y_ego').Values.Data;         % input: ego vehicle lateral position (m)
    y_ref = logsout.get('y_ref').Values.Data;         % input: reference lateral position (m)
    a_ego = logsout.get('a_ego').Values.Data;         % output: ego vehicle acceleration (m/s^2)
    delta = logsout.get('delta').Values.Data;         % output: steering angle (rad)

    % Convert the data into input and output cell arrays for training
    inputs_mat = [v_ego, y_ego, y_ref];
    inputs1_cell = num2cell(inputs_mat', 1);
    output1_cell = num2cell([a_ego, delta]', 1);
end