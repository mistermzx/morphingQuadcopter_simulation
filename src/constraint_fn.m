function [c, ceq] = constraint_fn(x, l, h, T_GE, T_fold)

    % Rename to sensible names
    s_BG_0 = [x(1); x(2); x(3)];
    Ds_BG_0 = [x(4); x(5); x(6)];
    T_f = x(7);

    % T_f must satisfy the dynamic equations
    g_G = T_GE*[0; 0; -9.81]; % gravity in G frame
    ceq = g_G(3)*(T_f^2)/2 + Ds_BG_0(3)*T_f + s_BG_0(3) - l/2;

    % We must pass throught the exit window at T_f
    c = zeros(4,1);
    c(1) = -(g_G(1)*(T_f^2)/2 + Ds_BG_0(1)*T_f + s_BG_0(1) + h/2);
    c(2) = g_G(1)*(T_f^2)/2 + Ds_BG_0(1)*T_f + s_BG_0(1) - h/2;

    % We can't hit the ceiling
    T_max = -Ds_BG_0(1)/g_G(1);
    if (T_max > T_fold) && (T_max < T_f)
        c(3) = g_G(1)*(T_max^2)/2 + Ds_BG_0(1)*T_max + s_BG_0(1) - h/2;
        c(4) = -(g_G(1)*(T_max^2)/2 + Ds_BG_0(1)*T_max + s_BG_0(1) + h/2);
    end

end