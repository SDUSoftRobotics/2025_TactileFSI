% Machine learning for paper titled: 
% "Tactile Perception through Fluid–Solid Interaction".
% Date: 2025-10-30
% Version: 1.0
% Author: Aida Parvaresh(aidap@mmmi.sdu.dk)
%
%
% Position and Force training and results of the tactile sensor using FCM.
% The full explanation on the model can be found in supplementary
% information S7. 
clc;
clear;
close all;

%% Select Sensor Type: '1D' or '2D'
sensorType = questdlg('Select the sensor type:', ...
                      'Sensor Selection', ...
                      '1D', '2D', '1D');  % Default is 1D

%% Select Data File
[filename, filepath] = uigetfile({'*.xlsx;*.xls','Excel Files (*.xlsx, *.xls)'}, ...
                                 ['Select the dataset for ' sensorType ' sensor']);
fullFile = fullfile(filepath, filename);
[~, name, ~] = fileparts(filename);
outputFilename = [name, '_ANFIS'];

%% Ask for number of clusters
switch sensorType
    case '1D'
        prompt = {'Enter number of clusters for X:', 'Enter number of clusters for F:'};
        dlgtitle = 'Cluster Settings (1D)';
        dims = [1 35];
        definput = {'4','10'};  % default values
        answer = inputdlg(prompt, dlgtitle, dims, definput);
        nCluster_X = str2double(answer{1});
        nCluster_F = str2double(answer{2});
        clusterSetup = {'X','F'};
        
    case '2D'
        prompt = {'Enter number of clusters for X:', 'Enter number of clusters for Y:', 'Enter number of clusters for F:'};
        dlgtitle = 'Cluster Settings (2D)';
        dims = [1 35];
        definput = {'4','4','10'};  % default values
        answer = inputdlg(prompt, dlgtitle, dims, definput);
        nCluster_X = str2double(answer{1});
        nCluster_Y = str2double(answer{2});
        nCluster_F = str2double(answer{3});
        clusterSetup = {'X','Y','F'};
end

%% Load Data
switch sensorType
    case '1D'
        data = readcell(fullFile);
        X = cell2mat(data(2:end, 1));
        Pl = cell2mat(data(2:end, 2));
        Pr = cell2mat(data(2:end, 3));
        F = cell2mat(data(2:end, 4));
        Inputs = [Pl Pr];
        Targets = [X F];

    case '2D'
        data = readcell(fullFile);
        X = cell2mat(data(2:end, 1));
        Y = cell2mat(data(2:end, 2));
        Tl = cell2mat(data(2:end, 3));
        Tr = cell2mat(data(2:end, 4));
        Pl = cell2mat(data(2:end, 5));
        Pr = cell2mat(data(2:end, 6));
        F = cell2mat(data(2:end, 7));
        Inputs = [Pl Pr Tl Tr];
        Targets = [X Y F];
end

%% Split Data into Train & Test
nData = size(Inputs, 1);
pTrain = 0.8;
nTrainData = round(pTrain * nData);
rng(7); 
randIndices = randperm(nData);

trainIndices = randIndices(1:nTrainData);
testIndices = randIndices(nTrainData+1:end);

TrainInputs = Inputs(trainIndices, :);
TrainTargets = Targets(trainIndices, :);
TestInputs = Inputs(testIndices, :);
TestTargets = Targets(testIndices, :);

%% FIS Design (Using FCM clustering)
FCMOptions = [2 150 1e-5 1];

switch sensorType
    case '1D'
        fis_X = genfis3(TrainInputs, TrainTargets(:,1), 'sugeno', nCluster_X, FCMOptions);
        fis_F = genfis3(TrainInputs, TrainTargets(:,2), 'sugeno', nCluster_F, FCMOptions);
        fisModels = {fis_X, fis_F};
        
    case '2D'
        fis_X = genfis3(TrainInputs, TrainTargets(:,1), 'sugeno', nCluster_X, FCMOptions);
        fis_Y = genfis3(TrainInputs, TrainTargets(:,2), 'sugeno', nCluster_Y, FCMOptions);
        fis_F = genfis3(TrainInputs, TrainTargets(:,3), 'sugeno', nCluster_F, FCMOptions);
        fisModels = {fis_X, fis_Y, fis_F};
end

%% Train ANFIS Models
TrainOptions = [200 0 0.01 0.9 1.1]; % [Epochs, ErrorGoal, StepSize, DecRate, IncRate]
OptimizationMethod = 1; % Hybrid

for i = 1:length(fisModels)
    fprintf('Training model for %s...\n', clusterSetup{i});
    fisModels{i} = anfis([TrainInputs TrainTargets(:,i)], fisModels{i}, TrainOptions, OptimizationMethod);
end

%% Evaluate Models
TestOutputs = zeros(size(TestTargets));
for i = 1:length(fisModels)
    TestOutputs(:,i) = evalfis(fisModels{i}, TestInputs);
end

%% Regression Plots
for i = 1:length(fisModels)
    figure;
    plotregression(TestTargets(:,i), TestOutputs(:,i), ['Test Data - ' clusterSetup{i}]);
end

%% RMSE Calculation
TestErrors = TestTargets - TestOutputs;
TestRMSE = sqrt(mean(TestErrors.^2,1));
fprintf('Test RMSE: '); disp(TestRMSE);

%% Save Models
save([outputFilename, '_fisModels.mat'], 'fisModels');
for i = 1:length(fisModels)
    writeFIS(fisModels{i}, [outputFilename '_fis_' clusterSetup{i} '.fis']);
end

%% Load and Use Saved Models
useSavedModel = questdlg('Do you want to use saved models for prediction?', ...
                         'Model Selection', ...
                         'Yes', 'No', 'No');
if strcmp(useSavedModel, 'Yes')

    % Load saved FIS models
    [modelFile, modelPath] = uigetfile('*.mat', 'Select the saved FIS models file');
    loadedModels = load(fullfile(modelPath, modelFile));
    fisModels = loadedModels.fisModels;
    
    % Determine sensor type based on loaded models (1D or 2D) and get
    % inputs for prediction
    if length(fisModels) == 2
        sensorType = '1D';
        prompt = {'PL value:', 'PR value:'};
    else
        sensorType = '2D';
        prompt = {'PL value:', 'PR value:', 'TL value:', 'TR value:'};
    end
    
    dlgtitle = [sensorType ' Sensor Input'];
    dims = [1 35];
    answer = inputdlg(prompt, dlgtitle, dims, definput);
    inputData = str2double(answer)';
    
    % Make predictions 
    predictions = zeros(1, length(fisModels));
    for i = 1:length(fisModels)
        predictions(i) = evalfis(fisModels{i}, inputData);
    end
    
    fprintf('\n=== %s Sensor Prediction Results ===\n', sensorType);
    switch sensorType
        case '1D'
            fprintf('Input - PL: %.4f, PR: %.4f\n', inputData(1), inputData(2));
            fprintf('Predicted X: %.4f\n', predictions(1));
            fprintf('Predicted F: %.4f\n', predictions(2));
         
        case '2D'
            fprintf('Input - PL: %.4f, PR: %.4f, TL: %.4f, TR: %.4f\n', ...
                    inputData(1), inputData(2), inputData(3), inputData(4));
            fprintf('Predicted X: %.4f\n', predictions(1));
            fprintf('Predicted Y: %.4f\n', predictions(2));
            fprintf('Predicted F: %.4f\n', predictions(3));
          
    end
end