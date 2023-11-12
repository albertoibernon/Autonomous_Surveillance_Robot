function setTime(time_step)
    global time_i
    time_i = time_i + time_step;
    clc;
    fprintf('Time: %f',time_i)

end