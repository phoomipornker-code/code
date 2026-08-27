function D = fwd_cccv_step(V_OUT, I_OUT, Vac)
%FWD_CCCV_STEP  Drop-in replacement for the CC/CV + Switch in the screenshot.
%
%  Simulink:
%    Interpreted MATLAB Function  →  fwd_cccv_step(u(1),u(2),u(3))
%    u = [V_OUT; I_OUT; Vac]     sample time = 0.02
%    y = D in 0 .. 0.450  (connect to the same >= carrier as before)
%
%  Matches cv58-boost-v14-forward-pi-v4  (cascade, not min/switch of two duties).

    persistent st t_ms
    if isempty(st)
        st = reset_st();
        t_ms = 0;
    end
    if nargin < 3 || isempty(Vac)
        Vac = 220;
    end

    [duty, st] = tick(st, V_OUT, V_OUT, abs(I_OUT), Vac, t_ms);
    t_ms = t_ms + 20;
    D = duty / 1023;
end

function st = reset_st()
    st.mode = 0;
    st.duty = 0;
    st.integI = 0;
    st.integV = 0;
    st.iref = 5.0;
    st.t_enter = 0;
    st.t_cv_enter = 0;
    st.t_cv_exit = 0;
    st.t_full = 0;
end

function [duty_raw, st] = tick(st, vbat_filt, vbat_raw, ibat, vac, now_ms)
    dt = 0.02;
    DMAX = 460;
    I_CC = 5.0;
    V_CV = 57.60;

    if vac < 100
        st.duty = 0; st.integI = 0; st.integV = 0;
        duty_raw = 0;
        return
    end

    switch st.mode
        case 0
            st.duty = slew(80, st.duty, 1.5, 4);
            if ibat >= 0.35 || (now_ms - st.t_enter) >= 2000
                st.mode = 1; st.integI = 0; st.iref = I_CC;
            end
        case 1
            iRef = I_CC;
            if vbat_filt >= 56.40
                span = max(0.20, V_CV - 56.40);
                iRef = iRef * clampv((V_CV - vbat_filt) / span, 0.10, 1.0);
            end
            iRef = clampv(iRef, 0, I_CC);
            st.iref = iRef;
            iErr = iRef - ibat;
            [dDuty, st.integI] = pi_aw(iErr, 8.0, 35.0, dt, st.integI, -20, 25);
            if iErr > 0.8 && st.duty < 200 && vac >= 140
                dDuty = max(dDuty, 2.0);
            end
            st.duty = slew(clampv(st.duty + dDuty, 0, DMAX), st.duty, 3.0, 5.0);
            vPeak = max(vbat_raw, vbat_filt);
            if vbat_filt >= 57.30 || vPeak >= 57.30
                st = to_cv(st, ibat);
            elseif vbat_filt >= 57.10
                if st.t_cv_enter == 0, st.t_cv_enter = now_ms; end
                if (now_ms - st.t_cv_enter) >= 200
                    st = to_cv(st, ibat);
                end
            else
                st.t_cv_enter = 0;
            end
        case 2
            vErr = V_CV - vbat_filt;
            near = abs(vErr) <= 0.35;
            if abs(vErr) <= 0.12
                vErr = 0; st.integV = st.integV * 0.92;
            end
            [iReq, st.integV] = pi_aw(vErr, 0.70, 0.35, dt, st.integV, 0.0, 3.0);
            if near
                iReq = min(iReq, 1.0 + clampv(vErr / 0.35, 0.0, 1.0));
            end
            st.iref = slew(clampv(iReq, 0.0, 3.0), st.iref, 0.06, 0.06);
            iErr = st.iref - ibat;
            if near
                [dDuty, st.integI] = pi_aw(iErr, 4.0, 14.0, dt, st.integI, -6, 6);
                step = 0.6; up = 1.0; dn = 1.8;
            else
                [dDuty, st.integI] = pi_aw(iErr, 6.4, 22.75, dt, st.integI, -20, 15);
                step = 2.0; up = 2.0; dn = 3.5;
            end
            dDuty = clampv(dDuty, -step, step);
            dutyTarget = st.duty + dDuty;
            if abs(V_CV - vbat_filt) <= 0.12
                dutyTarget = st.duty; st.integI = st.integI * 0.95;
            end
            if vbat_filt > V_CV
                dutyTarget = dutyTarget - (0.6 + (vbat_filt - V_CV) * 3.5);
                st.integV = st.integV * 0.85;
            end
            st.duty = slew(clampv(dutyTarget, 0, DMAX), st.duty, up, dn);
            if vbat_filt <= 56.40
                if st.t_cv_exit == 0, st.t_cv_exit = now_ms; end
                if (now_ms - st.t_cv_exit) >= 5000
                    st.mode = 1; st.integI = 0; st.integV = 0;
                    st.iref = 0.5; st.t_full = 0; st.t_cv_exit = 0;
                end
            else
                st.t_cv_exit = 0;
            end
        otherwise
            st.duty = 0;
            if vbat_filt <= 53.60 && vac >= 140
                st.mode = 1; st.integI = 0; st.integV = 0; st.iref = 0.5;
            end
    end

    if ibat > (I_CC + 0.25)
        st.duty = st.duty - (2.0 + (ibat - I_CC) * 3.0);
        st.integI = st.integI * 0.8;
    end
    if vbat_filt > V_CV
        over = vbat_filt - V_CV;
        if st.mode == 2
            st.duty = st.duty - (0.5 + over * 2.5);
        else
            st.duty = st.duty - (2.0 + over * 8.0); st.integI = 0;
        end
    end
    st.duty = clampv(st.duty, 0, DMAX);
    duty_raw = st.duty;
end

function st = to_cv(st, ibat)
    st.mode = 2; st.integI = 0; st.integV = 0;
    st.iref = clampv(ibat, 0.3, 2.0); st.t_cv_enter = 0;
end

function [out, integ] = pi_aw(err, kp, ki, dt, integ, lo, hi)
    p = kp * err;
    iCand = integ + ki * err * dt;
    out = p + iCand;
    if out > hi
        out = hi;
    elseif out < lo
        out = lo;
    else
        integ = iCand;
    end
end

function y = slew(target, current, upStep, downStep)
    d = target - current;
    if d > upStep, y = current + upStep;
    elseif d < -downStep, y = current - downStep;
    else, y = target;
    end
end

function y = clampv(x, lo, hi)
    y = min(max(x, lo), hi);
end
