% Define global variables
global Ts T v0 y0 seed

% Set seed for random number generator
seed = randi([0, 1000]);

% Define the sample time (Ts) and simulation duration (T) in seconds
Ts = 0.1;
T = 1000;

% Generate training data
[inputs1_cell, output1_cell] = generateLaneKeepingTrace();
inputs_test = inputs1_cell;
output_test = output1_cell;
for i = 1:20
    [input1, output1] = generateLaneKeepingTrace();
    inputs1_cell = catsamples(inputs1_cell, input1, 'pad');
    output1_cell = catsamples(output1_cell, output1, 'pad');
end

%% Construct NN controller

S = [10, 10, 10];
mrac_net = feedforwardnet(S);
for i = 1:length(S)
    mrac_net.layers{i}.transferFcn = 'poslin';
end

mrac_net.layers{length(S)+1}.transferFcn = 'purelin';
mrac_net.inputs{1}.processFcns = {};
mrac_net.outputs{length(S)+1}.processFcns = {};

mrac_net = configure(mrac_net, inputs1_cell, output1_cell);
mrac_net.plotFcns = {'plotperform', 'plottrainstate', ...
    'ploterrhist', 'plotregression', 'plotresponse'};
mrac_net.trainFcn = 'trainlm';

[x_tot, xi_tot, ai_tot, t_tot] = ...
    preparets(mrac_net, inputs1_cell, output1_cell);
mrac_net.trainParam.epochs = 500;
mrac_net.trainParam.min_grad = 1e-10;
[mrac_net, tr] = train(mrac_net, x_tot, t_tot, xi_tot, ai_tot);

testoutseq = mrac_net(inputs_test);
testout = cell2mat(testoutseq);
figure
plot(testout, 'r');
hold on 
plot(cell2mat(output_test), 'b')
hold off

% Generate Simulink model
% gensim(mrac_net)

% Store weights to network
% weights(1) = mrac_net.IW(1,1);
% for i = 1:length(S)
%     weights(i+1) = mrac_net.LW(i+1,i);
% end
% bias = mrac_net.b;
% 
% network.weights = weights;
% network.bias = bias;
% save network

% Remove example file folder from MATLAB path and close Simulink model
rmpath(fullfile(matlabroot, 'examples', 'mpc', 'main'));