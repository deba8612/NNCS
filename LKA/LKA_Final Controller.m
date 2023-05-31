% Define the lateral vehicle dynamics parameters
Cf = 19000;
Cr = 33000;
m = 1575;
Iz = 2875;
lf = 1.2;
lr = 1.6;

% Set the longitudinal velocity in m/s
Vx = 15;
Ts= 0.1;
T=15;

% Call getCurvature function to obtain md
time = 0:Ts:T;
md = getCurvature(Vx, time);

% Generate one trace for LKA system
[inputs_cell, output_cell] = generateOnetraceLKA(md);

% Generate additional training data
inputs_test = inputs_cell;
output_test = output_cell;
for i = 1:20
    [input1, output1] = generateOnetraceLKA(md);
    inputs_cell = catsamples(inputs_cell, input1, 'pad');
    output_cell = catsamples(output_cell, output1, 'pad');
end

% Construct NN controller
S = [10, 10, 10];
lka_net = feedforwardnet(S);
for i = 1:length(S)
    lka_net.layers{i}.transferFcn = 'poslin';
end
lka_net.layers{length(S)+1}.transferFcn = 'purelin';
lka_net.inputs{1}.processFcns = {};
lka_net.outputs{length(S)+1}.processFcns = {};

lka_net = configure(lka_net, inputs_cell, output_cell);
lka_net.plotFcns = {'plotperform', 'plottrainstate', 'ploterrhist', 'plotregression', 'plotresponse'};
lka_net.trainFcn = 'trainlm';

% Align lengths of input and output sequences
minLength = min(length(inputs_cell), length(output_cell));
inputs_cell = inputs_cell(1:minLength);
output_cell = output_cell(1:minLength);

% Prepare training data for the network
[x_tot, xi_tot, ai_tot, t_tot] = preparets(lka_net, inputs_cell, output_cell);

lka_net.trainParam.epochs = 500;
lka_net.trainParam.min_grad = 1e-15;
lka_net.trainParam.mu_max = 1e30; % Adjusted maximum mu value
[lka_net, tr] = train(lka_net, x_tot, t_tot, xi_tot, ai_tot);


testoutseq = lka_net(inputs_test);
testout = cell2mat(testoutseq);
figure
plot(testout, 'r', 'DisplayName', 'Predicted Output');
hold on 
plot(cell2mat(output_test), 'b', 'DisplayName', 'True Output');
hold off
legend('Location', 'best')


% Generate Simulink model for the trained LKA network
gensim(lka_net)

% Store weights and biases to network
% weights = cell(length(S)+1, 1);
% for i = 1:length(S)
%     weights{i} = lka_net.LW{i, i};
% end
% bias = lka_net.b;
% 
% network.weights = weights;
% network.bias = bias;
% save network

function [inputs_cell, output_cell] = generateOnetraceLKA(md)
    global Ts T refTrajectory A B C G PredictionHorizon
    
    % Set the sample time, Ts, and simulation duration, T, in seconds.
    Ts = 0.1;
    T = 15;
    
    % Define the reference trajectory as a function handle.
    refTrajectory = @(t) 0.3*t; % Example reference trajectory
    
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
    A = [-(2*Cf+2*Cr)/m/Vx, -Vx-(2*Cf*lf-2*Cr*lr)/m/Vx; -(2*Cf*lf-2*Cr*lr)/Iz/Vx, -(2*Cf*lf^2+2*Cr*lr^2)/Iz/Vx];
    B = [2*Cf/m, 2*Cf*lf/Iz]';
    C = eye(2);
    G = ss(A, B, C, 0); 
    % Add a delay param in the system matrices
    
    % Set the prediction horizon.
    PredictionHorizon = 30;
    
    % Run the simulation.
    sim("mpcLKAsystem.slx")
    
    % Input and output data for training
    steeringAngle = logsout.get(1).Values.Data; % Input (steering angle)
    y = logsout.get(2).Values.Data; % Output (vehicle lateral position)
    
    % Apply the output constraint of -0.5 rad/s to 0.5 rad/s
    steeringAngle = max(min(steeringAngle, 0.5), -0.5);
    
    % Convert input and output data into cell arrays
    inputs_cell = num2cell(steeringAngle', 1);
    output_cell = num2cell(y', 1);
end
