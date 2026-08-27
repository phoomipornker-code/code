%BUILD_FORWARD_CCCV_BLOCKS
%  Simulink CC/CV wired like the screenshot (Gain/Integrator/Switch/Sat)
%  but cascade: voltage PI -> Iref, current PI -> dDuty -> UnitDelay+Add.
%  No MATLAB Function.
%
%    cd sim
%    build_forward_cccv_blocks

model = 'forward_cccv_blocks';
if bdIsLoaded(model), close_system(model, 0); end
if exist([model '.slx'], 'file'), delete([model '.slx']); end

new_system(model);
open_system(model);
set_param(model, 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.02', ...
    'StopTime', '180');

% ---- inputs ----
add_block('simulink/Sources/In1', [model '/V_OUT'], 'Position', [30 70 60 90]);
add_block('simulink/Sources/In1', [model '/I_OUT'], 'Position', [30 320 60 340]);
add_block('simulink/Discrete/Zero-Order Hold', [model '/ZOH_Vmeas'], ...
    'SampleTime', '0.02', 'Position', [80 68 130 92]);
add_block('simulink/Discrete/Zero-Order Hold', [model '/ZOH_Imeas'], ...
    'SampleTime', '0.02', 'Position', [80 318 130 342]);
add_line(model, 'V_OUT/1', 'ZOH_Vmeas/1');
add_line(model, 'I_OUT/1', 'ZOH_Imeas/1');

% ---- voltage PI -> Iref_CV (amps) ----
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
add_line(model, 'ZOH_Vmeas/1', 'Verr/2');
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

add_line(model, 'ZOH_Vmeas/1', 'RelayCV/1');
add_line(model, 'Sat_Iref/1', 'SwIref/1');
add_line(model, 'RelayCV/1', 'SwIref/2');
add_line(model, 'Icc/1', 'SwIref/3');

% ---- current PI -> dDuty (raw counts / tick) ----
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
add_line(model, 'ZOH_Imeas/1', 'Ierr/2');
add_line(model, 'Ierr/1', 'Kp_i/1');
add_line(model, 'Ierr/1', 'Ki_i/1');
add_line(model, 'Kp_i/1', 'ZOH_i/1');
add_line(model, 'Ki_i/1', 'Int_i/1');
add_line(model, 'ZOH_i/1', 'SumPI_i/1');
add_line(model, 'Int_i/1', 'SumPI_i/2');
add_line(model, 'SumPI_i/1', 'Sat_dD/1');

% ---- Unit Delay + Add accumulates duty (NOT Memory inherit, NOT Ts/(z-1)) ----
add_block('simulink/Math Operations/Add', [model '/AddDuty'], ...
    'Inputs', '++', 'Position', [900 120 930 150]);
add_block('simulink/Discontinuities/Saturation', [model '/SatRaw'], ...
    'UpperLimit', '460', 'LowerLimit', '0', 'Position', [960 122 1000 148]);
add_block('simulink/Discrete/Unit Delay', [model '/MemDuty'], ...
    'SampleTime', '0.02', 'X0', '0', ...
    'Position', [960 175 1000 205]);
add_block('simulink/Math Operations/Gain', [model '/ToFrac'], ...
    'Gain', '1/1023', 'Position', [1040 125 1100 145]);
add_block('simulink/Discontinuities/Saturation', [model '/SatD'], ...
    'UpperLimit', '0.45', 'LowerLimit', '0', 'Position', [1140 122 1180 148]);
add_block('simulink/Sinks/Out1', [model '/Duty'], 'Position', [1230 125 1260 145]);
add_block('simulink/Sinks/Display', [model '/DutyDisp'], 'Position', [1230 165 1290 195]);

add_line(model, 'Sat_dD/1', 'AddDuty/1');
add_line(model, 'MemDuty/1', 'AddDuty/2');
add_line(model, 'AddDuty/1', 'SatRaw/1');
add_line(model, 'SatRaw/1', 'MemDuty/1');
add_line(model, 'SatRaw/1', 'ToFrac/1');
add_line(model, 'ToFrac/1', 'SatD/1');
add_line(model, 'SatD/1', 'Duty/1');
add_line(model, 'SatD/1', 'DutyDisp/1');

% ---- PWM compare like the screenshot ----
add_block('simulink/Sources/Repeating Sequence', [model '/Duty_Cy'], ...
    'rep_seq_t', '[0 1/67000]', 'rep_seq_y', '[0 1]', ...
    'Position', [1140 230 1210 270]);
add_block('simulink/Logic and Bit Operations/Relational Operator', ...
    [model '/GE'], 'Operator', '>=', 'Position', [1240 200 1270 250]);
add_block('simulink/Sinks/Out1', [model '/PWM'], 'Position', [1320 215 1350 235]);
add_line(model, 'SatD/1', 'GE/1');
add_line(model, 'Duty_Cy/1', 'GE/2');
add_line(model, 'GE/1', 'PWM/1');

save_system(model);
fprintf(['Created %s.slx (standard blocks only).\n' ...
    'Wire plant V_OUT / I_OUT into the two Inports.\n' ...
    'Feed the plant from Duty (0-0.45), NOT from PWM.\n' ...
    'On a switching plant, low-pass I_OUT before the I_OUT inport.\n'], model);
