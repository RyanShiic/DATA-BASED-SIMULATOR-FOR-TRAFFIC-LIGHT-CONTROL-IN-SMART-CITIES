function [NewPathArcs] = Update_Path(arc,d,p_i_v,lastPath)

    % This function is to update the path that the vehicle will go.
    % When a vehicle is approching to a traffic light, we compute the updated
    % path by adding the current traffic light waiting time (WaitingTime) to
    % the consideration and compute a optimal path for the time.

    % "d" is the destination
    % "p_i_v" is the speed of the vehicle
    
    % "TLvalue": value of the current traffic light.
    % "TLclock": clock of the current traffic light.
%     value(16)=0;value(17)=0;
% tao_t(16)=850;tao_t(17)=5;
% u_T(16)=1000;u_T(17)=1000;
% u_rho(16)=0.1;u_rho(17)=0.1;
% lastPath = 11;

    global N_v Net iniPathNodes iniPathArcs startNode endNode Nodes2Arc TLindex N_a Arc Lambada value tao_t u_T u_rho
    
    % we update the "weights" for each arc by replacing "NumofCells"
    % with "the time that a vehicle is expected to pass the road"    
    Net_prime = Net;
    Net_prime.Edges.Weight = Net.Edges.Weight/p_i_v;
    
    % We update the time to arrive the destination for each lane in the current arc          
    for j=1:Arc{3,arc}
        lambada = Lambada(j,arc);
        tlindex = TLindex(j,arc);
        TLvalue = value(tlindex);
        TLclock = tao_t(tlindex);
        % First, we calculate the traffic light waiting time
        if TLvalue == 1
            TLwaitingTime = 0;
        else
            TLwaitingTime = u_T(tlindex)*(1-u_rho(tlindex))- TLclock;
        end
        

        % Now, we add the "TLwaitingTime" to the weights of all NEXT possible roads (lambada) 
        [a,~]=size(lambada);
        No=zeros(a,2);
        n=zeros(1,a+1);
        for i=1:a
            No(i,1)=startNode(lambada(i,2));
            No(i,2)=endNode(lambada(i,2));
            [~,n(i+1)]=ismember([No(i,1),No(i,2)],Net_prime.Edges.EndNodes,'rows');
            if n(i+1)== n(i)
            else
                Net_prime.Edges.Weight(n(i+1)) = Net_prime.Edges.Weight(n(i+1)) + TLwaitingTime;
            end
        end   
    
    end
    
    % the vehicle's next road cannot be the same as the road they just
    % passed(by giving a huge weight to the last path)
    if lastPath==0
    else
        NO(i,1)=startNode(lastPath);
        NO(i,2)=endNode(lastPath);
        [~,r]=ismember([NO(i,1),NO(i,2)],Net_prime.Edges.EndNodes,'rows');
        Net_prime.Edges.Weight(r) = Net_prime.Edges.Weight(r) + 999999;
    end

    
 % plot(Net_prime,'EdgeLabel',Net_prime.Edges.Weight)

        % Finally, we compute the updated path base on "Net_prime"
        NewPathNodes = shortestpath(Net_prime,No(1,1),d);
        NewPathArcs = zeros(1,length(NewPathNodes)-1);
        for j=1:(length(NewPathNodes)-1)
            NewPathArcs(j) = Nodes2Arc(NewPathNodes(j),NewPathNodes(j+1))  ; 
        end
end