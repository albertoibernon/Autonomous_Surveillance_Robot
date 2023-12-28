function dist_sonic = deteccion_colisiones()

%% Ultrasonidos (Detección de Obstáculos)
dist_sonic_middle = apoloGetUltrasonicSensor('uc0');
dist_sonic_left   = apoloGetUltrasonicSensor('ul1');
dist_sonic_right  = apoloGetUltrasonicSensor('ur1');

dist_sonic = [dist_sonic_left dist_sonic_middle dist_sonic_right];

end