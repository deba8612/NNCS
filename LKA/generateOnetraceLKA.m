function [inputs_cell, output_cell] = generateOnetraceLKA()
    global Ts T refTrajectory A B C G PredictionHorizon

    % Set the sample time, Ts, and simulation duration, T, in seconds.
    Ts = 0.1;
    T = 15;

    % Define the reference trajectory as a function handle.
    refTrajectory = @(t) 0.3 * t; % Example reference trajectory

    % Specify the lateral vehicle dynamics parameters.
    m = 1575;
    Iz = 2875;
    lf = 1.2;
    lr = 1.6;
    Cf = 19000;
    Cr = 33000;

    % Specify the longitudinal velocity in m/s.
    Vx = 15;

    % Calculate the system matrices A and B based on the given parameters.
    A = [-(2 * Cf + 2 * Cr) / m / Vx, -Vx - (2 * Cf * lf - 2 * Cr * lr) / m / Vx; -(2 * Cf * lf - 2 * Cr * lr) / Iz / Vx, -(2 * Cf * lf^2 + 2 * Cr * lr^2) / Iz / Vx];
    B = [2 * Cf / m, 2 * Cf * lf / Iz]';
    C = eye(2);
    G = ss(A, B, C, 0);

    % Add a delay param in the system matrices

    % Set the prediction horizon.
    PredictionHorizon = 30;

    % Generate the time vector
    time = 0:Ts:T;

    % Run the simulation.
    sim("mpcLKAsystem.slx")

    % Calculate the curvature using the generated time vector
    curvatureStruct = getCurvature(Vx, time);
    curvature = curvatureStruct.signals.values';

    relative_yaw_angle = logsout.get(3).Values.Data; % input1
    lateral_deviation = logsout.get(2).Values.Data; % input2
    
    % Ensure all inputs have the same size
    
    Vx = Vx * ones(size(relative_yaw_angle));

    curvature = curvature .* ones(size(relative_yaw_angle));
    lateral_deviation = lateral_deviation .* ones(size(relative_yaw_angle));

    inputs_mat = [Vx, curvature, lateral_deviation, relative_yaw_angle];
    inputs_cell = num2cell(inputs_mat', 1);

    steering_angle = logsout.get(1).Values.Data; % output1
    output_cell = num2cell(steering_angle', 1);
end
