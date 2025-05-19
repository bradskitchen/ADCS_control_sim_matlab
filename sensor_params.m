%% Sensor Parameters
function [MagScaleBias, MagFieldBias, MagScaleNoise, MagFieldNoise, AngScaleBias, AngFieldBias, AngScaleNoise, AngFieldNoise] = sensor_params()
MagScaleBias = 1000; %%nT    
MagFieldBias = MagScaleBias*(2*rand() - 1);

MagScaleNoise = 250;
MagFieldNoise = MagScaleNoise*(2*rand() - 1);

AngScaleBias = .001; %%rad/s    
AngFieldBias = AngScaleBias*(2*rand() - 1);

AngScaleNoise = .0005; %%rad/s
AngFieldNoise = AngScaleNoise*(2*rand() - 1);
end