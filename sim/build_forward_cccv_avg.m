%BUILD_FORWARD_CCCV_AVG
%  Closed-loop stock-block model: cascade PI + average-value plant.
%  No 67 kHz PWM in the current loop. Press Run.
%
%    cd sim
%    build_forward_cccv_avg

model = 'forward_cccv_avg';
if bdIsLoaded(model), close_system(model, 0); end
if exist([model '.slx'], 'file'), delete([model '.slx']); end

new_system(model);
open_system(model);
set_param(model, 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.02', ...
    'StopTime', '30');

% ---- voltage PI -> Iref_CV ----
add_block('simulink/Sources/Constant', [model '/Vref'], ...
    'Value', '57.60', 'Position', [30 30 80 50]);
add_block('simulink/Math Operations/Sum', [model '/Verr'], ...
    'Inputs', '+-', 'Position', [120 45 140 75]);
add_block('simulink/Math Operations/Gain', [model '/Kp_v'], ...
    'Gain', '0.70', 'Position', [180 20 230 40]);
add_block('simulink/Discrete/Zero-Order Hold', [model '/ZOH_v'], ...
    'SampleTime', '0.02', 'Position', [250 18 300 42]);
add_block('simulink/Math Operations/Gain', [model '/Ki_v'], ...
    'Gain', '0.35', 'Position', [180 70 230 90]);
add_block('simulink/Discrete/Discrete Integrator', [model '/Int_v'], ...
    'gainval', '1', 'SampleTime', '0.02', ...
    'IntegratorMethod', 'Integration: Forward Euler', ...
    'Position', [250 62 300 98]);
add_block('simulink/Math Operations/Sum', [model '/SumPI_v'], ...
    'Inputs', '++', 'Position', [330 40 350 80]);
add_block('simulink/Discontinuities/Saturation', [model '/Sat_Iref'], ...
    'UpperLimit', '3', 'LowerLimit', '0', 'Position', [380 48 420 72]);

add_line(model, 'Vref/1', 'Verr/1');
add_line(model, 'Verr/1', 'Kp_v/1');
add_line(model, 'Verr/1', 'Ki_v/1');
add_line(model, 'Kp_v/1', 'ZOH_v/1');
add_line(model, 'Ki_v/1', 'Int_v/1');
add_line(model, 'ZOH_v/1', 'SumPI_v/1');
add_line(model, 'Int_v/1', 'SumPI_v/2');
add_line(model, 'SumPI_v/1', 'Sat_Iref/1');

% ---- Relay + Switch selects Iref ----
add_block('simulink/Discontinuities/Relay', [model '/RelayCV'], ...
    'OnSwitchValue', '57.10', 'OffSwitchValue', '56.40', ...
    'OnOutputValue', '1', 'OffOutputValue', '0', ...
    'Position', [180 160 230 200]);
add_block('simulink/Sources/Constant', [model '/Icc'], ...
    'Value', '5', 'Position', [380 200 420 220]);
add_block('simulink/Signal Routing/Switch', [model '/SwIref'], ...
    'Threshold', '0.5', 'Position', [470 80 520 160]);

add_line(model, 'Sat_Iref/1', 'SwIref/1');
add_line(model, 'RelayCV/1', 'SwIref/2');
add_line(model, 'Icc/1', 'SwIref/3');

% ---- current PI -> dDuty ----
add_block('simulink/Math Operations/Sum', [model '/Ierr'], ...
    'Inputs', '+-', 'Position', [560 115 580 145]);
add_block('simulink/Math Operations/Gain', [model '/Kp_i'], ...
    'Gain', '8', 'Position', [610 90 660 110]);
add_block('simulink/Discrete/Zero-Order Hold', [model '/ZOH_i'], ...
    'SampleTime', '0.02', 'Position', [680 88 730 112]);
add_block('simulink/Math Operations/Gain', [model '/Ki_i'], ...
    'Gain', '35', 'Position', [610 140 660 160]);
add_block('simulink/Discrete/Discrete Integrator', [model '/Int_i'], ...
    'gainval', '1', 'SampleTime', '0.02', ...
    'IntegratorMethod', 'Integration: Forward Euler', ...
    'Position', [680 132 730 168]);
add_block('simulink/Math Operations/Sum', [model '/SumPI_i'], ...
    'Inputs', '++', 'Position', [760 110 780 150]);
add_block('simulink/Discontinuities/Saturation', [model '/Sat_dD'], ...
    'UpperLimit', '25', 'LowerLimit', '-20', 'Position', [810 118 850 142]);

add_line(model, 'SwIref/1', 'Ierr/1');
add_line(model, 'Ierr/1', 'Kp_i/1');
add_line(model, 'Ierr/1', 'Ki_i/1');
add_line(model, 'Kp_i/1', 'ZOH_i/1');
add_line(model, 'Ki_i/1', 'Int_i/1');
add_line(model, 'ZOH_i/1', 'SumPI_i/1');
add_line(model, 'Int_i/1', 'SumPI_i/2');
add_line(model, 'SumPI_i/1', 'Sat_dD/1');

% ---- Unit Delay + Add (Ts = 0.02, not Memory inherit) ----
add_block('simulink/Math Operations/Add', [model '/AddDuty'], ...
    'Inputs', '++', 'Position', [900 120 930 150]);
add_block('simulink/Discontinuities/Saturation', [model '/SatRaw'], ...
    'UpperLimit', '460', 'LowerLimit', '0', 'Position', [960 122 1000 148]);
add_block('simulink/Discrete/Unit Delay', [model '/MemDuty'], ...
    'SampleTime', '0.02', 'X0', '0', 'Position', [960 175 1000 205]);
add_block('simulink/Math Operations/Gain', [model '/ToFrac'], ...
    'Gain', '1/1023', 'Position', [1040 125 1100 145]);
add_block('simulink/Discontinuities/Saturation', [model '/SatD'], ...
    'UpperLimit', '0.45', 'LowerLimit', '0', 'Position', [1140 122 1180 148]);

add_line(model, 'Sat_dD/1', 'AddDuty/1');
add_line(model, 'MemDuty/1', 'AddDuty/2');
add_line(model, 'AddDuty/1', 'SatRaw/1');
add_line(model, 'SatRaw/1', 'MemDuty/1');
add_line(model, 'SatRaw/1', 'ToFrac/1');
add_line(model, 'ToFrac/1', 'SatD/1');

% ---- average-value plant (same numbers as run_forward_sim.m) ----
% i_ss = sat(5 * Duty / 0.22, 0, 8)
% I[k] = I[k-1] + 0.5 * (i_ss - I[k-1])     % tau_i = 0.04, Ts = 0.02
% v_nl += I * Ts / 250                       % C_eq = 5 / 0.02
% V = v_nl + I * 0.04
add_block('simulink/Math Operations/Gain', [model '/IssGain'], ...
    'Gain', '5/0.22', 'Position', [1220 122 1280 148]);
add_block('simulink/Discontinuities/Saturation', [model '/SatIss'], ...
    'UpperLimit', '8', 'LowerLimit', '0', 'Position', [1310 122 1350 148]);
add_block('simulink/Math Operations/Sum', [model '/IerrP'], ...
    'Inputs', '+-', 'Position', [1390 120 1410 150]);
add_block('simulink/Math Operations/Gain', [model '/Ialpha'], ...
    'Gain', '0.5', 'Position', [1440 125 1480 145]);
add_block('simulink/Math Operations/Add', [model '/Iadd'], ...
    'Inputs', '++', 'Position', [1520 120 1550 150]);
add_block('simulink/Discrete/Unit Delay', [model '/Idelay'], ...
    'SampleTime', '0.02', 'X0', '0', 'Position', [1520 175 1560 205]);
add_block('simulink/Discrete/Discrete Integrator', [model '/Vnl'], ...
    'gainval', '0.004', 'SampleTime', '0.02', ...
    'InitialCondition', '54', ...
    'IntegratorMethod', 'Integration: Forward Euler', ...
    'Position', [1600 110 1660 160]);
add_block('simulink/Math Operations/Gain', [model '/Resr'], ...
    'Gain', '0.04', 'Position', [1600 180 1650 200]);
add_block('simulink/Math Operations/Add', [model '/Vsum'], ...
    'Inputs', '++', 'Position', [1700 130 1730 180]);

add_line(model, 'SatD/1', 'IssGain/1');
add_line(model, 'IssGain/1', 'SatIss/1');
add_line(model, 'SatIss/1', 'IerrP/1');
add_line(model, 'Ialpha/1', 'Iadd/1');
add_line(model, 'Idelay/1', 'Iadd/2');
add_line(model, 'Iadd/1', 'Idelay/1');
add_line(model, 'Iadd/1', 'Vnl/1');
add_line(model, 'Iadd/1', 'Resr/1');
add_line(model, 'Vnl/1', 'Vsum/1');
add_line(model, 'Resr/1', 'Vsum/2');
add_line(model, 'IerrP/1', 'Ialpha/1');

% feedback (average V and I — no PWM)
add_line(model, 'Vsum/1', 'Verr/2');
add_line(model, 'Vsum/1', 'RelayCV/1');
add_line(model, 'Iadd/1', 'Ierr/2');
add_line(model, 'Idelay/1', 'IerrP/2');

add_block('simulink/Signal Routing/Mux', [model '/MuxScope'], ...
    'Inputs', '4', 'Position', [1780 80 1785 220]);
add_block('simulink/Sinks/Scope', [model '/Scope'], 'Position', [1840 130 1870 160]);
add_line(model, 'Vsum/1', 'MuxScope/1');
add_line(model, 'Iadd/1', 'MuxScope/2');
add_line(model, 'SwIref/1', 'MuxScope/3');
add_line(model, 'SatD/1', 'MuxScope/4');
add_line(model, 'MuxScope/1', 'Scope/1');

save_system(model);
fprintf(['Created %s.slx — press Run.\n' ...
    'Scope: V, I, Iref, Duty(0-0.45). I should track 5 A, Duty not square.\n'], model);
