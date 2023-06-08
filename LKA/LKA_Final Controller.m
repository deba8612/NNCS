% Set the longitudinal velocity in m/s
Vx = 15;
Ts = 0.1;
T = 15;
m = 1575;
Iz = 2875;
lf = 1.2;
lr = 1.6;
Cf = 19000;
Cr = 33000;


% Call getCurvature function to obtain md
time = 0:Ts:T;
md = getCurvature(Vx, time);

% Run the simulation.
mdl = 'mpcLKAsystem';
open_system(mdl)


% Generate training data
[inputs_cell, output_cell] = generateOnetraceLKA();
inputs_test = inputs_cell;
output_test = output_cell;
for i = 1:20
    [input1, output1] = generateOnetraceLKA();
    inputs_cell = catsamples(inputs_cell, input1, 'pad');
    output_cell = catsamples(output_cell, output1, 'pad');
end


% Construct NN controller
S = [10, 10, 10];
lka_net = feedforwardnet(S);
for j = 1:length(S)
    lka_net.layers{j}.transferFcn = 'poslin';
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

lka_net.trainParam.epochs = 50;
lka_net.trainParam.min_grad = 1e-30;
lka_net.trainParam.lr= 0.01;
lka_net.trainParam.mu_max = 1e150; % Adjusted maximum mu value
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
