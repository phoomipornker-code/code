%RUN_FORWARD_SIM  Average-value Forward + firmware PI (20 ms).
%  Does not need Simulink. Plots Vbat, Ibat, Iref, duty, mode.
%
%  Plant is a simple 16S LFP pack + first-order current, for control
%  checkout only — not a switching-loss model.

clear; clc;

Ts = 0.02;
Tstop = 180;                 % seconds — use Q_Ah=5 so CC→CV fits in 3 min
N = round(Tstop / Ts);
Vac = 220;                   % measured AC bus (firmware MIN_AC = 140)
R_esr = 0.040;
tau_i = 0.04;                % current plant time constant (s)
C_eq = 5.0 / 0.02;           % 5 A raises ~0.02 V/s so CC→CV fits in 3 min
v_nl = 54.0;                 % rest voltage at t=0
vbat = v_nl;
ibat = 0;

st = forward_pi_reset();

t = (0:N-1).' * Ts;
V = zeros(N,1); I = zeros(N,1); Iref = zeros(N,1);
D = zeros(N,1); Mode = zeros(N,1);

for k = 1:N
    now_ms = (k-1) * Ts * 1000;
    v_filt = vbat;           % sim has no ADC noise; both channels equal
    [duty, st] = forward_pi_tick(st, v_filt, vbat, abs(ibat), Vac, now_ms);

    Dfrac = duty / 1023;     % firmware PWM compare uses 10-bit
    i_ss = max(0, min(5.0 * (Dfrac / 0.22), 8));  % ~5 A near duty 225
    ibat = ibat + Ts * (i_ss - ibat) / tau_i;
    v_nl = v_nl + ibat * Ts / C_eq;
    vbat = v_nl + ibat * R_esr;

    V(k) = vbat; I(k) = ibat; Iref(k) = st.iref;
    D(k) = duty; Mode(k) = st.mode;
end

figure('Name', 'Forward PI v4 (firmware tick)');
subplot(4,1,1); plot(t, V); ylabel('Vbat (V)'); grid on
yline(57.60, '--'); yline(57.10, ':'); title('Firmware Forward PI — not the old Switch diagram')
subplot(4,1,2); plot(t, I, t, Iref); ylabel('A'); legend('Ibat','Iref'); grid on
subplot(4,1,3); plot(t, D); ylabel('duty raw'); ylim([0 480]); grid on
subplot(4,1,4); stairs(t, Mode); ylabel('mode'); ylim([-0.2 3.2])
yticks(0:3); yticklabels({'SOFT','CC','CV','DONE'}); xlabel('t (s)'); grid on

fprintf('End: V=%.2f V  I=%.2f A  duty=%.0f  mode=%d\n', V(end), I(end), D(end), Mode(end));
