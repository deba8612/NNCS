    
% Set the sample time, Ts, and simulation duration, T, in seconds.
    Ts = 0.1;
    T = 15;

    % Set the longitudinal velocity in m/s.
    Vx = 15+1.5-3*rand(1);

    % Calculate the system matrices A and B based on the given parameters.
    m = 1575; % mass of the vehicle (kg)
    Iz = 2875;%
    lf = 1.2;
    lr = 1.6;
    Cf = 19000;
    Cr = 33000;

    A = [-(2 * Cf + 2 * Cr) / m / Vx, -Vx - (2 * Cf * lf - 2 * Cr * lr) / m / Vx; ...
         -(2 * Cf * lf - 2 * Cr * lr) / Iz / Vx, -(2 * Cf * lf^2 + 2 * Cr * lr^2) / Iz / Vx];
    B = [2 * Cf / m, 2 * Cf * lf / Iz]';
    C = eye(2);
    G = ss(A, B, C, 0);

    % Set the prediction horizon.
    PredictionHorizon = 30;

    % Generate the time vector.
    time = 0:Ts:T;
    md = getCurvature(Vx, time);

    % Run the simulation.
    sim("mpcLKAsystem.slx")

    % Retrieve simulation data.
    seq = logsout.get(1).Values.Data;%Curvature values,input
    relative_yaw_angle = logsout.get(4).Values.Data;%Relative Yaw Angle,input
    lateral_deviation = logsout.get(3).Values.Data;%Lateral Deviation,input
    steering_angle = logsout.get(2).Values.Data;%Steering Angle, output

    % Ensure all inputs have the same size.
    minLength = min([length(seq), length(relative_yaw_angle), length(lateral_deviation), length(steering_angle)]);
    relative_yaw_angle = relative_yaw_angle(1:minLength);
    lateral_deviation = lateral_deviation(1:minLength);

    % Create inputs cell array for scalar inputs.
    inputs_cell_scalar = [Vx * ones(minLength, 1), lateral_deviation, relative_yaw_angle];
    disp(inputs_cell_scalar)

    % Create inputs cell array for sequence input (seq).
    inputs_cell_seq = num2cell(seq);
    disp(inputs_cell_seq);

    % Concatenate the two cells vertically (along rows) to get the final inputs_cell.
    inputs_cell_combined = [seq, inputs_cell_scalar];
    inputs_cell = num2cell(inputs_cell_combined);
   

    % Prepare the output cell array for steering angles.
    output_cell = num2cell(steering_angle);
    disp(output_cell)

