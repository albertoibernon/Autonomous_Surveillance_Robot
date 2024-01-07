function [time,pos_real,odometry_mes,odometry_cor,P_acumulado,C_acumulado,comando_acu,...
     up_acu,ui_acu,ud_acu,error_pos_acu,dist_sonic_left_acu,dist_sonic_middle_acu,...
     dist_sonic_right_acu] = acumular(time,pos_real,odometry_mes,odometry_cor,P_acumulado, ...
     C_acumulado,pos_real_i,odometry_mes_i,Xk,Pk,correccion,comando_acu,up_acu,ui_acu,...
     ud_acu,error_pos_acu,dist_sonic_left_acu,dist_sonic_middle_acu,dist_sonic_right_acu,...
     comando,up,ui,ud,error_pos,dist_sonic)

    global time_i

    %% Acumular variables

    time                  = [time; time_i];
    pos_real              = [pos_real; pos_real_i];
    odometry_mes          = [odometry_mes; odometry_mes_i];
    odometry_cor          = [odometry_cor; Xk];
    P_acumulado           = [P_acumulado; Pk];
    C_acumulado           = [C_acumulado;correccion];
    comando_acu           = [comando_acu comando];
    up_acu                = [up_acu up]; 
    ui_acu                = [ui_acu ui]; 
    ud_acu                = [ud_acu ud]; 
    error_pos_acu         = [error_pos_acu error_pos]; 
    dist_sonic_left_acu   = [dist_sonic_left_acu    dist_sonic(1)]; 
    dist_sonic_middle_acu = [dist_sonic_middle_acu  dist_sonic(2)]; 
    dist_sonic_right_acu  = [dist_sonic_right_acu   dist_sonic(3)];

end