function [inside] = Dset(X)

global T_c n N u_rho u_T n_v N_v
    
%% ------------ Extract state variables:tao_c,tao_t, value, tao_v, p_i_v ------------
    tao_c = X(1);
    
    %  The state variales of traffic lights starts from the 4-th varialbe to (2*N+3)-th variable of X  
    S=zeros(n,N);
    for i=1:N
        S(:,i)= ExtractState(n,N,i,X(4:2*N+3,:));
    end
    
    tao_t=zeros(1,N);
    value=zeros(1,N);
    for i=1:N
        tao_t(i) = S(1,i);
        value(i) = S(2,i);
    end
    
    %  Extract the state variales of vehicles start from the 4+2*N varialbe of X
    S_v=zeros(n_v,N_v);
    for i=1:N_v
        S_v(:,i)= ExtractState(n_v,N_v,i,X(4+2*N:end-1,:));
    end
    
    tao_v = zeros(1,N_v);
    p_i_v = zeros(1,N_v);
    for i=1:N_v
        tao_v(i) = S_v(1,i);
        p_i_v(i) = S_v(2,i);
    end
    
%% ------------ Define Dset (If one system is in Dset then all systems are in Dset)------------
    inside = 0;
    
    % Check if the controller is in Dset 
    if (tao_c >= T_c)
        inside = 1;
    else
        
        % Check if there is a vehicle in Dset
        for j=1:N_v
            if tao_v(j) >= 1/p_i_v(j)
                inside = 1;
                break
            end
        end
        
        %     Check if every traffic light is in Dset
        %     when the light is "green", but the clock is greater or equal than
        %     the "green period", or the light is "red", but the clock is
        %     greater or equal than the "red period"
        if inside==0
            for i=1:N
                if ((value(i)>=1) && (tao_t(i) >= u_rho(i) * u_T(i))) || ((value(i)<=0) && (tao_t(i) >= (1-u_rho(i)) * u_T(i)))
                    inside=1;
                    break
                end
            end
        end
        
    end
     
   
end