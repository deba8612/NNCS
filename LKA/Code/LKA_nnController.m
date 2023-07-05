
% Convert inputs_cell and output_cell to MATLAB arrays
inputs_seq = cell2mat(inputs_cell(:, 1:11));  % Assuming the first 11 columns are for 'seq'
inputs_scalar = cell2mat(inputs_cell(:, 12:end));
output_flat = cell2mat(output_cell)';

% Randomly shuffle the indices
num_samples = size(inputs_seq, 1);
indices = randperm(num_samples);

% Split the data into training and testing sets (80% training, 20% testing)
train_size = round(0.8 * num_samples);
trainInd = indices(1:train_size);
testInd = indices(train_size+1:end);

inputs_seq_train = inputs_seq(trainInd, :);
inputs_seq_test = inputs_seq(testInd, :);
inputs_scalar_train = inputs_scalar(trainInd, :);
inputs_scalar_test = inputs_scalar(testInd, :);
output_train = output_flat(trainInd);
output_test = output_flat(testInd);

% Create and configure the neural network
num_hidden_units1= 20;
num_hidden_units2 = 20;
net = fitnet([num_hidden_units1, num_hidden_units2,1],'trainlm');

% Set activation functions for hidden layers
net.layers{1}.transferFcn = 'logsig'; % sigmoid for LSTM-like behavior
net.layers{2}.transferFcn = 'purelin';
net.layers{3}.transferFcn = 'purelin';

% Prepare the training data
inputs_seq_train = inputs_seq_train';
inputs_scalar_train = inputs_scalar_train';
output_train = output_train';

% Train the neural network
net.trainParam.epochs = 200;
net = train(net, [inputs_seq_train; inputs_scalar_train], output_train');

% Prepare the test data
inputs_seq_test = inputs_seq_test';
inputs_scalar_test = inputs_scalar_test';
output_test = output_test';

% Make predictions on the test data
predictions = net([inputs_seq_test; inputs_scalar_test]);

% Calculate Mean Squared Error (MSE) on the test data
MSE = immse(predictions, output_test');
disp(['Mean Squared Error on Test Data: ', num2str(MSE)]);

% Plot true vs. predicted steering angles
figure;
plot(output_test','b');
hold on;
plot(predictions, 'r--', 'LineWidth', 2);
legend('True', 'Predicted');
xlabel('Time');
ylabel('Steering Angle');
title('True vs. Predicted Steering Angle');
grid on;
gensim(net)
