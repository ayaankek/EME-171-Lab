function dXdt = eqns(t, X)
    global mcr Jcr ksf ksr bsf bsr mtf mtr ktf ktr Lcg_default Lwb g vc ...

    % State variables
    omega_p = X(1);    % Pitch angular velocity
    vh = X(2);         % Heave velocity
    qsf = X(3);        % Front suspension displacement
    qsr = X(4);        % Rear suspension displacement
    ptf = X(5);        % Front tire momentum
    ptr = X(6);        % Rear tire momentum
    qtf = X(7);        % Front tire deflection
    qtr = X(8);        % Rear tire deflection

    % Kinematic relationships (small angle assumption)
    vsf = vh + Lcg_default * omega_p;  % Front suspension velocity
    vsr = vh - (Lwb - Lcg_default) * omega_p;  % Rear suspension velocity

    % Suspension forces and state derivatives
    dXdt = zeros(8, 1);

    % Equations of motion
    dXdt(1) = Lcg_default * (qsf * ksf + bsf * (ptf / mtf - vh / mcr - Lcg_default * omega_p)) ...
             - (Lwb - Lcg_default) * (qsr * ksr + bsr * (ptr / mtr - vh / mcr + (Lwb - Lcg_default) * omega_p)) / Jcr;

    dXdt(2) = -mcr * g + ksf * qsf + bsf * (ptf / mtf - vh / mcr - Lcg_default * omega_p) ...
              + ksr * qsr + bsr * (ptr / mtr - vh / mcr + (Lwb - Lcg_default) * omega_p);

    dXdt(3) = (ptf / mtf - vh / mcr - Lcg_default * omega_p);  % qsf rate of change
    dXdt(4) = (ptr / mtr - vh / mcr + (Lwb - Lcg_default) * omega_p);  % qsr rate of change

    dXdt(5) = ktf * qtf - mtf * g - ksf * qsf - bsf * (ptf / mtf - vh / mcr - Lcg_default * omega_p);  % ptf rate
    dXdt(6) = ktr * qtr - mtr * g - ksr * qsr - bsr * (ptr / mtr - vh / mcr + (Lwb - Lcg_default) * omega_p);  % ptr rate

    dXdt(7) = vc;  % Front tire vertical input velocity
    dXdt(8) = vc;  % Rear tire vertical input velocity
end
