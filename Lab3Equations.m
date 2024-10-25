function dydt = eqns(t, y)
    % eqns.m 

    global vc Lcg_1 Lcg_2 m_cr r_gy J_cr k_sf k_sr b_sf b_sr m_tf m_tr k_tf k_tr L_wb g A delta_max L bump_dist t_front_start t_front_apex t_front_end t_rear_start t_rear_apex t_rear_end;

    % Define state variables
    p_J = y(1);             % Pitch angular momentum
    p_cr = y(2);            % Vertical momentum of cycle and rider
    q_sf = y(3);            % Front suspension spring displacement
    q_sr = y(4);            % Rear suspension spring displacement
    p_tf = y(5);            % Momentum of front tire mass
    p_tr = y(6);            % Momentum of rear tire mass
    q_tf = y(7);            % Front tire deflection
    q_tr = y(8);            % Rear tire deflection

    % Input: road input velocities based on bump profile
    v_fi = road_input_velocity(t, t_front_start, t_front_apex, t_front_end, A);
    v_ri = road_input_velocity(t, t_rear_start, t_rear_apex, t_rear_end, A);

    % Calculate state derivatives
    a = Lcg_1;   % use standard or forward CG distance based on configuration
    b = L_wb - a;

    % Calculate state derivatives using equations of motion
    dydt = zeros(8,1);
    dydt(1) = a * (q_sf * k_sf + b_sf * (p_tf / m_tf - p_cr / m_cr - a * p_J / J_cr)) - ...
              b * (q_sr * k_sr + b_sr * (p_tr / m_tr - p_cr / m_cr + b * p_J / J_cr));
    dydt(2) = -m_cr * g + q_sf * k_sf + b_sf * (p_tf / m_tf - p_cr / m_cr - a * p_J / J_cr) + ...
              q_sr * k_sr + b_sr * (p_tr / m_tr - p_cr / m_cr + b * p_J / J_cr);
    dydt(3) = (p_tf / m_tf - p_cr / m_cr - a * p_J / J_cr);
    dydt(4) = (p_tr / m_tr - p_cr / m_cr + b * p_J / J_cr);
    dydt(5) = q_tf * k_tf - m_tf * g - q_sf * k_sf - b_sf * (p_tf / m_tf - p_cr / m_cr - a * p_J / J_cr);
    dydt(6) = q_tr * k_tr - m_tr * g - q_sr * k_sr - b_sr * (p_tr / m_tr - p_cr / m_cr + b * p_J / J_cr);
    dydt(7) = v_fi - (p_tf / m_tf);
    dydt(8) = v_ri - (p_tr / m_tr);
end

% Road input velocity as a function of time, using the bump profile
function v = road_input_velocity(t, t_start, t_apex, t_end, A)
    if t < t_start
        v = 0;
    elseif t >= t_start && t < t_apex
        v = (2 * A / (t_apex - t_start)) * (t - t_start);
    elseif t >= t_apex && t < t_end
        v = -(2 * A / (t_end - t_apex)) * (t - t_end);
    else
        v = 0;
    end
end
