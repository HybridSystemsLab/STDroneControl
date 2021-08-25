function lpfData = lpf_2_init(sample_fr, cutt_fr, start_sample)
    %% Function Purpose:
    %   Initializing the low pas filter struct
    
    %% Input
    % sample_fr: Frequency at which the data is being smapled
    % cutt_fr: Cutt off frequency
    % start_sample: Initial sample point
    
    %% Output
    % lpfData: A struct containing the information and paramets for the low pass filter.
    
    %%
    fr = sample_fr/cutt_fr;
    ohm = tan(pi/fr);
    c = 1 + 2*cos(pi/4)*ohm + ohm*ohm;
    
    lpfData.b0 = ohm*ohm/c;
    lpfData.b1 = 2*lpfData.b0;
    lpfData.b2 = lpfData.b0;
    
    lpfData.a1 = 2*(ohm*ohm - 1)/c;
    lpfData.a2 = (1 - 2*cos(pi/4)*ohm + ohm*ohm)/c;
    
    lpfData.delay1 = start_sample;
    lpfData.delay2 = start_sample;
end