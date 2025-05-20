%% Sensor Parameters
function [MagScaleBias, MagFieldBias, MagScaleNoise, MagFieldNoise, AngScaleBias, AngFieldBias, AngScaleNoise, AngFieldNoise] = sensor_params()
MagScaleBias = 4e-7; %%T    
MagFieldBias = MagScaleBias*(2*rand() - 1);

MagScaleNoise = 1e-5;
MagFieldNoise = MagScaleNoise*(2*rand() - 1);

AngScaleBias = .01; %%rad/s    
AngFieldBias = AngScaleBias*(2*rand() - 1);

AngScaleNoise = .001; %%rad/s
AngFieldNoise = AngScaleNoise*(2*rand() - 1);
end