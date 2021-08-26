function [dX] = FlowMap(X)

    global  N N_v

    % CONTROLLER: Clocks keep counting and eta stays the same value
    %     ( does not jump to the next interval)
    
    dX=[1;0;0];
    
    % TRAFFIC LIGHTS: Clocks keep counting and traffic lights stay same values
    for i=1:N
        dX=[dX;[1;0]];
    end
    
    % VEHICLES: Clocks keep counting and all other variables stay the same values
    for i=1:N_v
        dX=[dX;[1;0;0;0;0;0;0]];
    end
    
    % QUEUE LENGTH: Stays the same value
    dX=[dX;0];
end