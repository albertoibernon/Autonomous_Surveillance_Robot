function [u,up,ui,ud] = pidController(error, Kp, Ki, Kd)
%% Controlador PID basado en la distania entre la referencia y la posición 
%% estimada (error) y proporcionando la velocidad de rotación
   global integralState prevError;

    if isempty(integralState)
        integralState = 0;
        prevError = 0;
    end

    integralState = integralState + error;
    derivative = error - prevError;

    up = Kp * error;         % Proporcional
    ui = Ki * integralState; % Integrador
    ud = Kd * derivative;    % Derivativo

    u = up + ui + ud;

    prevError = error;
end