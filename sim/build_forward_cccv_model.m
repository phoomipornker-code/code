%BUILD_FORWARD_CCCV_MODEL
%  Creates Simulink model forward_cccv_v4 that replaces the screenshot:
%  old = two duty PIs + Switch(V>55.8)
%  new = one MATLAB Function (firmware cascade) + same PWM compare.
%
%  Run in MATLAB with Simulink:
%    cd sim
%    build_forward_cccv_model

model = 'forward_cccv_v4';
if bdIsLoaded(model)
    close_system(model, 0);
end
if exist([model '.slx'], 'file')
    delete([model '.slx']);
end

new_system(model);
open_system(model);
set_param(model, 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.02', ...
    'StopTime', '180', 'FixedStepAuto', 'off');

% Mux: V_OUT, I_OUT, Vac
add_block('simulink/Sources/In1', [model '/V_OUT'], 'Position', [40 40 70 60]);
add_block('simulink/Sources/In1', [model '/I_OUT'], 'Position', [40 90 70 110]);
add_block('simulink/Sources/Constant', [model '/Vac'], ...
    'Value', '220', 'Position', [40 140 80 160]);
add_block('simulink/Signal Routing/Mux', [model '/MuxU'], ...
    'Inputs', '3', 'Position', [130 70 135 150]);

add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', ...
    [model '/CCCV'], ...
    'MATLABFcn', 'fwd_cccv_step(u(1),u(2),u(3))', ...
    'Position', [190 80 330 140]);

add_block('simulink/Discontinuities/Saturation', [model '/SatD'], ...
    'UpperLimit', '0.45', 'LowerLimit', '0', 'Position', [370 95 410 125]);
add_block('simulink/Sinks/Out1', [model '/Duty'], 'Position', [460 100 490 120]);
add_block('simulink/Sinks/Display', [model '/DutyDisp'], 'Position', [460 140 520 170]);
add_block('simulink/Sinks/Scope', [model '/ScopeDuty'], 'Position', [460 50 490 80]);

add_line(model, 'V_OUT/1', 'MuxU/1');
add_line(model, 'I_OUT/1', 'MuxU/2');
add_line(model, 'Vac/1', 'MuxU/3');
add_line(model, 'MuxU/1', 'CCCV/1');
add_line(model, 'CCCV/1', 'SatD/1');
add_line(model, 'SatD/1', 'Duty/1');
add_line(model, 'SatD/1', 'DutyDisp/1');
add_line(model, 'SatD/1', 'ScopeDuty/1');

% Keep the old PWM compare as a reminder (optional subsystem)
add_block('simulink/Sources/Repeating Sequence', [model '/Carrier67k'], ...
    'rep_seq_t', '[0 1/67000]', 'rep_seq_y', '[0 1]', ...
    'Position', [370 210 430 250]);
add_block('simulink/Logic and Bit Operations/Relational Operator', ...
    [model '/GE'], 'Operator', '>=', 'Position', [470 200 500 240]);
add_block('simulink/Sinks/Out1', [model '/PWM'], 'Position', [540 210 570 230]);
add_line(model, 'SatD/1', 'GE/1');
add_line(model, 'Carrier67k/1', 'GE/2');
add_line(model, 'GE/1', 'PWM/1');

save_system(model);
fprintf(['Created %s.slx\n' ...
    'Connect plant: V_OUT and I_OUT from your battery/forward model.\n' ...
    'Duty (0-0.45) and PWM replace the old Switch output.\n'], model);
