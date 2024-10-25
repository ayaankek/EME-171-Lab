 % Lab3eqns.m 
    function ds = eqns(t, s)

    global vc Lcg_1 Lcg_2 m_cr r_gy J_cr k_sf k_sr b_sf b_sr m_tf m_tr k_tf k_tr L_wb g A delta_max L bump_dist t_front_start t_front_apex t_front_end t_rear_start t_rear_apex t_rear_end;

    % State variables

    p_J = s(1);             % Pitch angular momentum
    p_cr = s(2);            % Vertical momentum of cycle and rider
    q_sf = s(3);            % Front suspension spring displacement
    q_sr = s(4);            % Rear suspension spring displacement
    p_tf = s(5);            % Momentum of front tire mass
    p_tr = s(6);            % Momentum of rear tire mass
    q_tf = s(7);            % Front tire deflection
    q_tr = s(8);            % Rear tire deflection

    % Input

    v_fi = road_input_velocity(t, t_front_start, t_front_apex, t_front_end, A);
    v_ri = road_input_velocity(t, t_rear_start, t_rear_apex, t_rear_end, A);

    % State derivatives

    a = Lcg_1;   % use standard or forward CG distance based on configuration
    b = L_wb - a;

    % State derivatives using equations of motion

    %ds = zeros(8,1);
    p_J_dot = a * (q_sf * k_sf + b_sf * (p_tf / m_tf - p_cr / m_cr - a * p_J / J_cr)) - b * (q_sr * k_sr + b_sr * (p_tr / m_tr - p_cr / m_cr + b * p_J / J_cr));
    p_cr_dot = -m_cr * g + q_sf * k_sf + b_sf * (p_tf / m_tf - p_cr / m_cr - a * p_J / J_cr) + q_sr * k_sr + b_sr * (p_tr / m_tr - p_cr / m_cr + b * p_J / J_cr);
    q_sf_dot = (p_tf / m_tf - p_cr / m_cr - a * p_J / J_cr);
    q_sr_dot = (p_tr / m_tr - p_cr / m_cr + b * p_J / J_cr);
    p_tf_dot = q_tf * k_tf - m_tf * g - q_sf * k_sf - b_sf * (p_tf / m_tf - p_cr / m_cr - a * p_J / J_cr);
    p_tr_dot = q_tr * k_tr - m_tr * g - q_sr * k_sr - b_sr * (p_tr / m_tr - p_cr / m_cr + b * p_J / J_cr);
    q_tf_dot = v_fi - (p_tf / m_tf);
    q_tr_dot = v_ri - (p_tr / m_tr);

    ds = [p_J_dot; p_cr_dot; q_sf_dot; q_sr_dot; p_tf_dot; p_tr_dot; q_tf_dot; q_tr_dot];
end

% Road input velocity 

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
