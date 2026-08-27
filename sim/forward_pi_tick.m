function [duty_raw, st] = forward_pi_tick(st, vbat_filt, vbat_raw, ibat, vac, now_ms)
%FORWARD_PI_TICK  One 20 ms Forward tick matching cv58-boost-v14-forward-pi-v4.
%
%  duty_raw  LEDC counts 0..460
%  st        persistent controller state (see forward_pi_reset)

    dt = 0.02;
    DMAX = 460;
    I_CC = 5.0;
    V_CV = 57.60;

    if vac < 100
        st.duty = 0;
        st.integI = 0;
        st.integV = 0;
        duty_raw = 0;
        return
    end

    switch st.mode
        case 0  % SOFTSTART
            st.duty = apply_slew(80, st.duty, 1.5, 4);
            if ibat >= 0.35 || (now_ms - st.t_enter) >= 2000
                st.mode = 1;
                st.integI = 0;
                st.iref = I_CC;
            end

        case 1  % CC
            iRef = I_CC;
            if vbat_filt >= 56.40
                span = max(0.20, V_CV - 56.40);
                iRef = iRef * clampf((V_CV - vbat_filt) / span, 0.10, 1.0);
            end
            iRef = clampf(iRef, 0, I_CC);
            st.iref = iRef;
            iErr = iRef - ibat;
            [dDuty, st.integI] = run_pi(iErr, 8.0, 35.0, dt, st.integI, -20, 25);
            if iErr > 0.8 && st.duty < 200 && vac >= 140
                dDuty = max(dDuty, 2.0);
            end
            st.duty = apply_slew(clampf(st.duty + dDuty, 0, DMAX), st.duty, 3.0, 5.0);

            vPeak = max(vbat_raw, vbat_filt);
            if vbat_filt >= 57.30 || vPeak >= 57.30
                st = enter_cv(st, ibat);
            elseif vbat_filt >= 57.10
                if st.t_cv_enter == 0
                    st.t_cv_enter = now_ms;
                end
                if (now_ms - st.t_cv_enter) >= 200
                    st = enter_cv(st, ibat);
                end
            else
                st.t_cv_enter = 0;
            end

        case 2  % CV
            vErr = V_CV - vbat_filt;
            near = abs(vErr) <= 0.35;
            if abs(vErr) <= 0.12
                vErr = 0;
                st.integV = st.integV * 0.92;
            end
            [iReq, st.integV] = run_pi(vErr, 0.70, 0.35, dt, st.integV, 0.0, 3.0);
            if near
                nearCap = 1.0 + clampf(vErr / 0.35, 0.0, 1.0) * 1.0;
                iReq = min(iReq, nearCap);
            end
            iReq = clampf(iReq, 0.0, 3.0);
            st.iref = apply_slew(iReq, st.iref, 0.06, 0.06);

            iErr = st.iref - ibat;
            kp = 8.0 * (0.50 * near + 0.80 * ~near);
            ki = 35.0 * (0.40 * near + 0.65 * ~near);
            lo = -6.0 * near + -20.0 * ~near;
            hi =  6.0 * near +  15.0 * ~near;
            [dDuty, st.integI] = run_pi(iErr, kp, ki, dt, st.integI, lo, hi);
            step = 0.6 * near + 2.0 * ~near;
            dDuty = clampf(dDuty, -step, step);

            dutyTarget = st.duty + dDuty;
            if abs(V_CV - vbat_filt) <= 0.12
                dutyTarget = st.duty;
                st.integI = st.integI * 0.95;
            end
            if vbat_filt > V_CV
                dutyTarget = dutyTarget - (0.6 + (vbat_filt - V_CV) * 3.5);
                st.integV = st.integV * 0.85;
            end
            dutyTarget = clampf(dutyTarget, 0, DMAX);
            up = 1.0 * near + 2.0 * ~near;
            dn = 1.8 * near + 3.5 * ~near;
            st.duty = apply_slew(dutyTarget, st.duty, up, dn);

            if vbat_filt <= 56.40
                if st.t_cv_exit == 0
                    st.t_cv_exit = now_ms;
                end
                if (now_ms - st.t_cv_exit) >= 5000
                    st.mode = 1;
                    st.integI = 0;
                    st.integV = 0;
                    st.iref = 0.5;
                    st.t_full = 0;
                    st.t_cv_exit = 0;
                end
            else
                st.t_cv_exit = 0;
            end

            if vbat_filt >= 57.40 && ibat <= 0.50
                if st.t_full == 0
                    st.t_full = now_ms;
                end
                if (now_ms - st.t_full) >= 60000
                    st.mode = 3;
                    st.duty = 0;
                end
            else
                st.t_full = 0;
            end

        otherwise  % DONE
            st.duty = 0;
            if vbat_filt <= 53.60 && vac >= 140
                st.mode = 1;
                st.integI = 0;
                st.integV = 0;
                st.iref = 0.5;
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
            st.duty = st.duty - (2.0 + over * 8.0);
            st.integI = 0;
        end
    end
    if vbat_filt >= 58.10 || vbat_raw >= 58.10
        st.duty = min(st.duty, 120);
    end
    st.duty = clampf(st.duty, 0, DMAX);
    duty_raw = st.duty;
end

function st = enter_cv(st, ibat)
    st.mode = 2;
    st.integI = 0;
    st.integV = 0;
    st.iref = clampf(ibat, 0.3, 2.0);
    st.t_cv_enter = 0;
end

function [out, integ] = run_pi(err, kp, ki, dt, integ, lo, hi)
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

function y = apply_slew(target, current, upStep, downStep)
    d = target - current;
    if d > upStep
        y = current + upStep;
    elseif d < -downStep
        y = current - downStep;
    else
        y = target;
    end
end

function y = clampf(x, lo, hi)
    y = min(max(x, lo), hi);
end
