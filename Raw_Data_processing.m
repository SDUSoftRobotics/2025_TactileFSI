
% Raw data processing for paper titled: 
% "Tactile Perception through Fluid–Solid Interaction".
% Date: 2025-10-30
% Version: 1.0
% Author: Arman Goshtasbi (argo@mmmi.sdu.dk) 
%
%
% This code process the raw data collected from the experiment using the
% NI-DAQ to convert the raw voltage data into the pressure and force values. 
% The final output is called Final_data and output maximum left and right
% pressure, maximum force during the indentation, and left and right rise
% times. 

clear,clc,close all
%% Data Extraction and initialization



load("underwater\1D_underwater_50mm.mat")  % Load the data
 
sampling_freq = 100;
start_point = 1*sampling_freq;  % start point of data collection
end_point = 70*sampling_freq; % end point of data collection

% left pressure data extraction and filtering 
P_l = data.Dev1_ai1;
P_l_filt = lowpass(P_l,0.01,sampling_freq);
P_l_final = P_l_filt(start_point:end_point);

% right pressure data extraction and filtering 
P_r = data.Dev1_ai3;
P_r_filt = lowpass(P_r,0.01,sampling_freq);
P_r_final = P_r_filt(start_point:end_point);
Data_length = length(P_r_final);

% Force value extraction and filtering 
force = data.Dev1_ai0;
force_f = lowpass(force,0.01,sampling_freq);
force_f = -force_f(start_point:end_point);


layer_depth=40;
flag_f = 0;
cl = 0;
base = 1;


%% Finding the point where indentation happen 

for i=1:Data_length
    if cl 
        cl=cl-1;
        continue
    end
%change the threshold based on the measured forced value
    if (i+49<Data_length)&&(force_f(i+50)-force_f(i)>0.005)&&(force_f(i+50)-force_f(i)>0) 
        flag_f=flag_f+1;
        data.Time(i);
        index(flag_f) = i;
        cl=350;
    end 
end
index (end+1) = Data_length;


%% Removing the base value

% finding the base value
for k = 1:flag_f

    base_val_mean_l(k) =mean(P_l_final(index(k):index(k)+50));
    base_val_l(index(k):index(k+1)) = base_val_mean_l(k);

end
for k = 1:flag_f

    base_val_mean_r(k) =mean(P_r_final(index(k):index(k)+50));
    base_val_r(index(k):index(k+1)) = base_val_mean_r(k);

end

for k = 1:flag_f

    base_val_mean_f(k) =mean(force_f(index(k):index(k)+50));
    base_val_f(index(k):index(k+1)) = base_val_mean_f(k);

end
% 
base_val_l(1:index(1)-1) = base_val_l(index(1));
base_val_r(1:index(1)-1) = base_val_r(index(1));
base_val_f(1:index(1)-1) = base_val_f(index(1));

% removing the base value
prob_l_final = P_l_final-base_val_l';
prob_r_final = P_r_final-base_val_r';
prob_f_final = force_f - base_val_f';



%% Converting voltage to force and pressure value 

poke_range_l = [];
poke_range_r = [];
poke_range_f = [];

TR = zeros(flag_f,2);
for poke= 1:flag_f    
    poke_range_l = prob_l_final(index(poke):index(poke+1))*5;
    poke_range_r = prob_r_final(index(poke):index(poke+1))*5;
    poke_range_f = prob_f_final(index(poke):index(poke+1));

    P_l_proc(poke) = 5*max(prob_l_final(index(poke):index(poke+1))); % extracting maximum left pressure during indentation
    P_r_proc(poke) = 5*max(prob_r_final(index(poke):index(poke+1))); % extracting maximum right pressure during indentation
    force_proc(poke) = 4.1448*max(prob_f_final(index(poke):index(poke+1))); % extracting maximum force applied during indentation   
    TR_l = find(poke_range_l>0.95*P_l_proc(poke)); % finding 95% rise time for left pressure
    TR_r = find(poke_range_r>0.95*P_r_proc(poke));  % finding 95% rise time for right pressure
    TR(poke,:) = [TR_l(1),TR_r(1)];

end





Final_data = [P_l_proc',P_r_proc',force_proc',TR]; % The final data includes PL, PR, F_max, tl, and tr)