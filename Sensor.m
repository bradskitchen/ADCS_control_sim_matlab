function [BB, pqr] = Sensor(BB, pqr)

for i = 1:3
    %%get sensor params
    [MagScaleBias, MagFieldBias, MagScaleNoise, MagFieldNoise, AngScaleBias, AngFieldBias, AngScaleNoise, AngFieldNoise] = sensor_params();

    %%pollute the data
    BB(i) = BB(i) + MagFieldBias + MagFieldNoise;
    pqr(i) = pqr(i) + AngFieldBias + AngFieldNoise;
end