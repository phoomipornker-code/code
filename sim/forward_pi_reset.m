function st = forward_pi_reset()
%FORWARD_PI_RESET  Initial Forward PI state (SoftStart).
    st.mode = 0;          % 0 SOFT  1 CC  2 CV  3 DONE
    st.duty = 0;
    st.integI = 0;
    st.integV = 0;
    st.iref = 5.0;
    st.t_enter = 0;
    st.t_cv_enter = 0;
    st.t_cv_exit = 0;
    st.t_full = 0;
end
