function [Xp] = JumpMap(X)
    
global T_c n N u_rho u_T h dt N_v n_v Lambada TLindex Arc o d iniPathNodes iniPathArcs startNode endNode tao_t value lastPath qLength jiToInd kk S_0 NR
    
%%  ------------ Extract state variables: tao_c, tao_t, value, tao_v, p_i_v, p_i_aux, alpha_i, l_i, g_i, c_i ------------

    tao_c = X(1);
    eta = X(2);
    x_c_prime = X(3);
    
    % The state variales of traffic lights starts from the 4-th varialbe to (2*N+3)-th variable of X  
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
    p_i_aux = zeros(1,N_v);
    alpha_i = zeros(1,N_v);
    l_i = zeros(1,N_v);
    g_i = zeros(1,N_v);
    c_i = zeros(1,N_v);
    for i=1:N_v
        tao_v(i) = S_v(1,i);
        p_i_v(i) = S_v(2,i);
        p_i_aux(i) = S_v(3,i);
        alpha_i(i) = S_v(4,i);
        l_i(i) = S_v(5,i);
        g_i(i) = S_v(6,i);
        c_i(i) = S_v(7,i);
    end
    AveQLength = X(end);
    
%% ------------ check IF the controller in Dset ------------
    
    % During the jump set, tao is set to 0, if eta < h,eta = eta + 1 , 
    % if not eta = 1, and x_c_prime remains the same
    
    if tao_c >= T_c - dt
        tao_c_p = 0;
        % Check if the number of decision intervals exceed to the max
        if eta < h
            eta_p = eta + 1;
            
            % If the "day" is not over :        
            % Update duty cycles and cycle periods when jump to the next
    
%             % decision interval
%             for i=1:N
%                 u_rho(i)= u_rho(i)+ 0.1 ;
%                 u_T(i) = u_T(i) + 10;
%             end

            
        else
            eta_p = 1;     
                
%             % Update duty cycles and cycle periods when jump to the first
%             % decision interval( first inverval of the next "day")
%             u_rho(1) = 0.2;
%             u_T(1) = 30;
%             for i=2:N
%                 u_rho(i) = u_rho(i-1) + 0.01; 
%                 u_T(i) =u_T(i-1) + 10; 
%             end           
        end
        
        x_c_prime_p = x_c_prime;
    else
        tao_c_p = tao_c;
        eta_p = eta;
        x_c_prime_p = x_c_prime; 
    end
    
%% ------------ check IF one of the traffic lights is in Dset ------------

    tao_t_p=zeros(1,N);
    value_p=zeros(1,N);
    
    % check if every traffic light is in Dset. If it is, set clock to zero
    % and switch the traffic light's value
    for i=1:N
        if ((value(i)>=1) && (tao_t(i) >= u_rho(i) * u_T(i)- dt)) || ((value(i)<=0) && (tao_t(i) >= (1-u_rho(i)) * u_T(i)-dt))
            tao_t_p(i) = 0;          
            value_p(i) = not(value(i));
        else
            tao_t_p(i) = tao_t(i);
            value_p(i) = value(i);
        end
    end
    
%% ------------ check IF one of the vehicles is in Dset ------------

tao_v_p = zeros(1,N_v);
p_i_v_p = zeros(1,N_v);
p_i_aux_p = zeros(1,N_v);
alpha_i_p = zeros(1,N_v);
l_i_p = zeros(1,N_v);
g_i_p = zeros(1,N_v);
c_i_p = zeros(1,N_v);

for i=1:N_v
    if alpha_i(i)==0    % If the vehicle is out of the map
        tao_v_p(i)=0; p_i_v_p(i)=0; p_i_aux_p(i)=0; alpha_i_p(i)=0; l_i_p(i)=0; g_i_p(i)=0; c_i_p(i)=0;
%         arriveTime(i)=time;
    else
    
            % If i-th vehicle is in Dset
            if tao_v(i) >= 1/p_i_v(i)-dt

                % Define u_i_L (current values of traffic lights) and u_i_E (free cells in road r)
                tlIndex = TLindex(l_i(i), alpha_i(i));
                if tlIndex == 0
                    u_i_L = 1;
                else
                    u_i_L = value(tlIndex);            
                end

                lambada = Lambada(l_i(i), alpha_i(i));
                u_i_E = FreeCells(l_i(i), alpha_i(i), alpha_i, l_i, c_i); % Free cells in current lane
                u_i_Eminus = FreeCells(l_i(i)-1, alpha_i(i), alpha_i, l_i, c_i); % Free cells in (lane-1,alpha)
                u_i_Eplus = FreeCells(l_i(i)+1, alpha_i(i), alpha_i, l_i, c_i); % Free cells in (lane+1,alpha)
                if lambada(1)==0 % if the vehicle is on exiting roads (nextroad=[0 0])
                    next_road(2)= 0;
                    next_road(1)= 0;
                    u_i_Epsi = 1;
                else
                    if (c_i(i)>(Arc{2,alpha_i(i)}/2) && mod(c_i(i),3)==0)|| c_i(i)<=1 || tao_v(i) <= 1.5 
                        NewPathArcs = Update_Path(alpha_i(i),d(i),p_i_v(i),lastPath(i));                       
                        next_road = Psi(NewPathArcs, alpha_i(i), d(i));
                        NR{1,i}=next_road;
                    else
                         next_road = NR{1,i};
                    end
                    
                    if next_road(2)== 0 % if the vehicle is about to arrive the destination
                        u_i_Epsi = 1;
                    else
                        u_i_Epsi = FreeCells(next_road(1), next_road(2), alpha_i, l_i, c_i); % Free cells in Psi
                    end
                end

                % Set clock to 0 and p unchanged
                tao_v_p(i) = 0;
                p_i_v_p(i) = p_i_v(i);
                p_i_aux_p(i) = p_i_aux(i);

                % Jump map for alpha
                GAMMA = (alpha_i(i)==0)||(c_i(i)<Arc{2,alpha_i(i)})||(u_i_L==0)|| (not(ismember(1,u_i_Epsi)));
                                           % This is the set that vehicle dose not change arc (GAMMA)
                if GAMMA
                    alpha_i_p(i) = alpha_i(i);
                else
                    [Lia, ~] = ismember(next_road,Lambada(l_i(i),alpha_i(i)), 'rows');
                    if Lia == 1 || next_road(2)==0 % the current road can go to the "next road"
                        alpha_i_p(i) = next_road(2);
                        lastPath(i) = alpha_i(i);% Store the last path the vehicle passed
                    else        % the vehicle has failed to go to the target lane due to traffic congestion, so that we need to go to a backup road and calculate again
                        backupRoads = Lambada(l_i(i),alpha_i(i));
                        backupRoad = backupRoads(1,:);
                        alpha_i_p(i) = backupRoad(2);
                        lastPath(i) = alpha_i(i);% Store the last path the vehicle passed
                    end
                end

                % Jump map for l
                DELTA = (l_i(i)==0)||(l_i(i) == g_i(i))||((l_i(i) < g_i(i))&&(not(ismember(c_i(i)+1,u_i_Eplus))))||((l_i(i) > g_i(i))&&(not(ismember(c_i(i)+1,u_i_Eminus))));
                                           % This is the set that vehicle dose not change lane (DELTA)
                DELTA_plus = (l_i(i)>0 && g_i(i)>0) && (g_i(i)>l_i(i)) && (ismember(c_i(i)+1,u_i_Eplus));
                                           % This is the set that vehicle changes lane to a larger one (DELTA_plus)
                DELTA_minus = (l_i(i)>0 && g_i(i)>0) && (g_i(i)<l_i(i)) && (ismember(c_i(i)+1,u_i_Eminus));
                                           % This is the set that vehicle changes lane to a smaller one (DELTA_minus)
                if DELTA && GAMMA
                    l_i_p(i) = l_i(i);
                elseif DELTA_plus && GAMMA
                    l_i_p(i) = l_i(i) + 1;
                elseif DELTA_minus && GAMMA
                    l_i_p(i) = l_i(i) - 1;
                else
                    [Lia, ~] = ismember(next_road,Lambada(l_i(i),alpha_i(i)), 'rows');
                    if Lia == 1 || next_road(2)==0 % the current road can go to the "next road"
                        l_i_p(i) = next_road(1);
                    else        % the vehicle has failed to go to the target lane due to traffic congestion, so that we need to go to a backup road and calculate again
                        l_i_p(i) = backupRoad(1);
                    end
                end

                % Jump map for g
                if GAMMA
                    g_i_p(i) = Delta (l_i(i),alpha_i(i), next_road, c_i(i));
                else
                    [Lia, ~] = ismember(next_road,Lambada(l_i(i),alpha_i(i)), 'rows');
                    if Lia == 1 || next_road(2)==0 % the current road can go to the "next road"
                        g_i_p(i) = next_road(1);
                    else        % the vehicle has failed to go to the target lane due to traffic congestion, so that we need to go to a backup road and calculate again
                        g_i_p(i) = backupRoad(1);
                    end    
                end

                % Jump map for c
                if DELTA && GAMMA
                    c_i_p(i) = k(c_i(i),u_i_E);
                elseif DELTA_plus && GAMMA
                    c_i_p(i) = k(c_i(i),u_i_Eplus);
                elseif DELTA_minus && GAMMA
                    c_i_p(i) = k(c_i(i),u_i_Eminus);
                else
                    c_i_p(i) = 1;
                end

            else       
                tao_v_p(i) = tao_v(i);
                p_i_v_p(i) = p_i_v(i);
                p_i_aux_p(i) = p_i_aux(i);
                alpha_i_p(i) = alpha_i(i);
                l_i_p(i) = l_i(i);
                g_i_p(i) = g_i(i);
                c_i_p(i) = c_i(i);
            end
    end
end

%% Compute Queue length at each traffic light


    for i=1:N
        [a,b]=find(jiToInd==i);
        freeC = FreeCells(a, b, alpha_i_p, l_i_p, c_i_p);
        qLength(i)=Arc{2,b}-max(freeC);
    end
    
    AveQLength_p=mean(qLength(:));


%% ------------  the output of the function ------------ 
    Xp = [tao_c_p;eta_p;x_c_prime_p];
    
    for i=1:N
    Xp=[Xp;[tao_t_p(i);value_p(i)]];
    end
    
    for i=1:N_v
    Xp=[Xp;[tao_v_p(i); p_i_v_p(i); p_i_aux_p(i); alpha_i_p(i); l_i_p(i); g_i_p(i); c_i_p(i)]];
    end
    
    Xp=[Xp;AveQLength_p];

end
