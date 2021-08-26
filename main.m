tic
%% This part is to build the road network

clc;
close all
clear all

% Building structures to represent the network.
% "alpha" represent each arc.
% In each alpha:
% N: two nodes at each end of the arc (directed);
% E: number of cells in each arc;
% NUMofLANES: number of lanes in this arc;
% t: target lanes that current lane can reach, have the form;[lane1, arc1; lane2, arc2; ...].

% If the current lane and current arc are equal to 0, it means the vehicle
% is out of the map
global Arc Lambada Net N N_a N_n jiToInd

N_a = 23; %Num of ARCs
N_n = 10; %Num of NODEs(crossing)
alpha1 = struct('N',[1,2],'E',50,'NUMofLANES',3,'t1',[1,5;2,5;1,8],'t2',[1,5;2,5],'t3',[1,2;2,2;3,2]);
alpha2 = struct('N',[2,1],'E',50,'NUMofLANES',3,'t1',[1,1;2,1;3,1],'t2',[1,3],'t3',[1,3]);
alpha3 = struct('N',[1,3],'E',75,'NUMofLANES',1,'t1',[1,4;1,6;2,6;1,7;1,10;2,10]);
alpha4 = struct('N',[3,1],'E',75,'NUMofLANES',1,'t1',[1,3;1,1;2,1;3,1]);
alpha5 = struct('N',[2,3],'E',75,'NUMofLANES',2,'t1',[1,6;2,6;1,4],'t2',[1,7;1,10;2,10]);
alpha6 = struct('N',[3,2],'E',75,'NUMofLANES',2,'t1',[1,5;2,5;1,8],'t2',[1,2;2,2;3,2]);
alpha7 = struct('N',[3,4],'E',100,'NUMofLANES',1,'t1',[1,12]);
alpha8 = struct('N',[2,7],'E',60,'NUMofLANES',1,'t1',[1,22;1,20;1,17;1,9]);
alpha9 = struct('N',[7,2],'E',60,'NUMofLANES',1,'t1',[1,2;2,2;3,2;1,5;2,5;1,8]);
alpha10 = struct('N',[3,5],'E',50,'NUMofLANES',2,'t1',[1,11;2,11;1,15;2,15],'t2',[1,19;1,16]);
alpha11 = struct('N',[5,3],'E',50,'NUMofLANES',2,'t1',[1,6;2,6;1,10;2,10],'t2',[1,4;1,7]);
alpha12 = struct('N',[4,6],'E',40,'NUMofLANES',1,'t1',[1,23;1,14;2,14;1,13]);
alpha13 = struct('N',[6,4],'E',40,'NUMofLANES',1,'t1',[1,12]);
alpha14 = struct('N',[6,5],'E',100,'NUMofLANES',2,'t1',[1,15;2,15;1,19],'t2',[1,16;1,11;2,11]);
alpha15 = struct('N',[5,6],'E',100,'NUMofLANES',2,'t1',[1,14;2,14],'t2',[1,23;1,13]);
alpha16 = struct('N',[5,7],'E',60,'NUMofLANES',1,'t1',[1,9;1,22;1,20;1,17]);
alpha17 = struct('N',[7,5],'E',60,'NUMofLANES',1,'t1',[1,19;1,15;2,15;1,11;2,11]);
alpha18 = struct('N',[8,5],'E',50,'NUMofLANES',1,'t1',[1,16;1,19;1,11;2,11;1,15;2,15]);
alpha19 = struct('N',[5,8],'E',50,'NUMofLANES',1,'t1',[1,18;1,21]);
alpha20 = struct('N',[7,8],'E',750,'NUMofLANES',1,'t1',[1,21;1,18]);
alpha21 = struct('N',[8,7],'E',75,'NUMofLANES',1,'t1',[1,22;1,20;1,9;1,17]);
alpha22 = struct('N',[7,9],'E',90,'NUMofLANES',1,'t1',[0,0]);
alpha23 = struct('N',[6,10],'E',90,'NUMofLANES',1,'t1',[0,0]);

% Reconstruct structures as a cell array (cell in cell)
A=cell(1,N_a);
A{1,1}=struct2cell(alpha1);
A{1,2}=struct2cell(alpha2);
A{1,3}=struct2cell(alpha3);
A{1,4}=struct2cell(alpha4);
A{1,5}=struct2cell(alpha5);
A{1,6}=struct2cell(alpha6);
A{1,7}=struct2cell(alpha7);
A{1,8}=struct2cell(alpha8);
A{1,9}=struct2cell(alpha9);
A{1,10}=struct2cell(alpha10);
A{1,11}=struct2cell(alpha11);
A{1,12}=struct2cell(alpha12);
A{1,13}=struct2cell(alpha13);
A{1,14}=struct2cell(alpha14);
A{1,15}=struct2cell(alpha15);
A{1,16}=struct2cell(alpha16);
A{1,17}=struct2cell(alpha17);
A{1,18}=struct2cell(alpha18);
A{1,19}=struct2cell(alpha19);
A{1,20}=struct2cell(alpha20);
A{1,21}=struct2cell(alpha21);
A{1,22}=struct2cell(alpha22);
A{1,23}=struct2cell(alpha23);

% Assign empty cells for those who have different length
for i = 1:N_a
    if size(A{1,i})<6
        A{1,i} = [A{1,i};cell(6-max(size(A{1,i})),1)];
    end
end

% Combine all cells(arcs) together
Arc=[];
for i=1:N_a
    Arc = [Arc A{1,i}];
end

% If we find an empty cell, assign [0;0] to it
for i=1:numel(Arc)
    B=cell2mat(Arc(i)); 
    if isempty(B)
       Arc(i)={zeros(1,2)}; 
    end
end
[w, NUMofARCS]=size(Arc);

% Construct visible diagram of the network (use 'digraph')
s=[];
t=[];
E=[];
NUMofLANES=[];
% names=[];
for i=1:NUMofARCS
%     for j=1:Arc{3,i}
        s = [s Arc{1,i}(1)];
        t = [t Arc{1,i}(2)];
        E = [E Arc{2,i}];
    NUMofLANES = [NUMofLANES Arc{3,i}];
    
    
%     names{i} = ['Node' num2str(i)];
end
 Net = digraph(s,t,E);
 plot(Net,'EdgeLabel',Net.Edges.Weight)
%  Net.Edges.Properties.VariableNames{2} = 'E';   
%   ,'EdgeLabel',Net.Edges.Weight
 
% The function Lambada: is a mapping connecting incoming and outgoing
% roads. That is, the lanes and arcs that can be reached from the current
% lane and arc.
% The output of the function has the form of matrices. Different rows
% represent different combination of lane and arc.
Lambada = @(lane,arc) cell2mat(Arc(lane+3,arc));

%% Assign a traffic light to each non-exiting lane

global TLindex
N=0; %Num of traffic lights
jiToInd=zeros(3,NUMofARCS);
indx = 1:200;
for i=1:NUMofARCS
    for j=1:3
        % Check if current lane is non-exiting lane. If not, we assign a
        % traffic light to this lane
        if Lambada(j,i) == 0
            
        else
            N = N+1;
            jiToInd(j,i)=indx(N);
        end        
    end
end

% Functino TLindex mapping (lane arc) to the index of traffic lights
TLindex = @(lane,arc) jiToInd(lane,arc);

%% Define parameters and the state space

% First, define the number of decision intervals in which a day is divided 
% (h), and define the duration (in seconds) of each decision interval(T_c)
% n is the number of state of one traffic light
% N is the number of traffic lights
% n_v is the number of state of one vehicle
% N_v is the number of vehicles on the road network

global h T_c n n_v N_v
h = 3;
T_c = 3*70/h;
n = 2;
n_v = 7;
N_v = 3;

% Then define the input variables: u_c = (u_c_L, u_c_E, u_c_w)
% with u_c_L, the current value of the traffic light
%      u_c_E, free cells in road r
%      u_c_w, stochastic process for decision making.
% We have N roads in total
global u_c_L u_c_E u_c_w

%   Define the state space X, the state variable has the form:
%   X = (tao_c, eta, x_c_prime,   tao_t, value(first traffic lights), ...(other traffic lights),  tao_v, p_i_v, p_i_aux, alpha_i, l_i, g_i, c_i(frist vehicle), ...,AveQlength)

%   tao_c is the clock of the controller
%   variable eta keeps track of which interval of the day is currently on.
%   x_c_prime is the auxiliary parameter.

%   tao_t, value are the clock of the traffic lights and the traffic light
% values. In this example, we define the traffic light "green" as 1
% "red" as 0.

%   tao_v is the clock of vehicles.
%   p_i_v and p_i_aux are vehicle-spcific parameters, represent the
% velocity and the auxiliary parameter respectively of a vehicle.
%   alpha_i is the current arc.
%   l_i is the current lane.
%   g_i is the target lane.
%   c_i is the current position. (cell occupied by the vehicle on the current lane l_i)
%   AveQlength is the average queue length of all lanes

%% Randomly define origin nodes and destination nodes for each vehicle and compute shortest path between origins and destinations

global o d exitArcs exitNodes iniPathNodes iniPathArcs startNode endNode Nodes2Arc arriveTime S_0 NR qLength

% "o" is the origin nodes, "d" is the destination nodes

arriveTime=zeros(1,N_v);
% Functino startNode and endNode mapping an arc to the two nodes that are
% connected by the arc
startNode = @(arc) Arc{1,arc}(1);
endNode = @(arc) Arc{1,arc}(2);

% Generate another function that mapping two nodes to an arc 
Nodes2Arc = @(node1,node2) find((any(cellfun(@(x) isequal(x, [node1,node2]), Arc(1:2,:))))~=0);

o = zeros(1,N_v); 
d = zeros(1,N_v);
iniArc = zeros(1,N_v); % Intial arc for each vehicle

% Compute exiting Arcs
exitArcs = find(any(jiToInd)==0); 

% Compute exiting nodes
exitNodes = zeros(1,length(exitArcs)); 
for i=1:length(exitArcs)
    exitNodes(i)= endNode(exitArcs(i));
end

iniPathNodes = cell(N_v,1); % Construct a cell array to store the shortest path(nodes)
iniPathArcs = cell(N_v,1); % Construct a cell array to store the shortest path(arcs)
optTime=zeros(1,N_v);
totalCells=zeros(1,N_v);

for i=1:N_v     % set of origins and destinations for vehicle i
    
    % origin nodes (origins have to be different from exiting nodes)
    o(i) = unidrnd(N_n);
    while ismember(o(i),exitNodes)
        o(i) = unidrnd(N_n);
    end
    
    % randomly generate destination node (destination has to be different from origin)
    d(i) = unidrnd(N_n);
    while d(i)==o(i)
        d(i)=unidrnd(N_n);
    end   
    
    if i==1
       o(i)=7;d(i)=6;
    else
        o(i)=7;d(i)=8;
    end
    
    % Now we compute the shortest path between o(i) and d(i)
    iniPathNodes{i,1} = shortestpath(Net,o(i),d(i)); 
    
    % Next, we need to compute arc index alone the initial path
    for j=1:(length(iniPathNodes{i,1})-1)
        iniPathArcs{i,1}(j) = Nodes2Arc(iniPathNodes{i,1}(j),iniPathNodes{i,1}(j+1));   
    end
    
    %Compute the first Arc to go for each vehicle (iniArc)
    iniArc(i) = iniPathArcs{i,1}(1);
    
    %Total cells that ith vehicle will pass
    for kkk=1:length(iniPathArcs{i,1})
        totalCells(i)=totalCells(i)+Arc{2,iniPathArcs{i,1}(kkk)};
    end
    
end

NR = cell(1,N_v);
qLength = zeros(1,N);

%% Define initial state for the controller and traffic lights

%   Set the initial state to [0;1;0;0;1 ......]
X0 = [0;1;0]; % This is the initial state for the controller

% Pack initial states of every traffic light 
S_0=zeros(n,N);
for i=1:N
        S_0(:,i) = [0;0]; 
        % Set every clock to 0 and every value to "red"(0) at begining
end


% Traffic light constrains
% node 1
[S_0(2,5),S_0(2,6),S_0(2,4),S_0(2,8)]=deal(1);
% node 2
[S_0(2,2),S_0(2,3),S_0(2,12),S_0(2,1)]=deal(1);
[S_0(2,11),S_0(2,15)]=deal(0);
% node 3
[S_0(2,18),S_0(2,19)]=deal(1);
[S_0(2,7),S_0(2,10),S_0(2,9)]=deal(0);
% node 4
[S_0(2,13),S_0(2,21)]=deal(1);
% node 5
[S_0(2,16),S_0(2,17)]=deal(1);
[S_0(2,22),S_0(2,23),S_0(2,27),S_0(2,28)]=deal(0);
% node 6
[S_0(2,24),S_0(2,25)]=deal(1);
[S_0(2,20)]=deal(0);
% node 7
[S_0(2,14)]=deal(1);
[S_0(2,26),S_0(2,31)]=deal(0);
% node 8
[S_0(2,29),S_0(2,30)]=deal(1);

% %% Define u_rho, u_T of the first interval of day 1
% 
% % u_rho, u_T are the the duty cycle (portion of green) and the cycle period 
% % u_rho u_T for N traffic lights in day 1, period 1 
global u_rho u_T
% u_rho(1) = 0.2;
% u_T(1) = 30;
% for i=2:N
%     u_rho(i) = u_rho(i-1) + 0.01;  % the duty cycle (portion of green)
%     u_T(i) =u_T(i-1) + 10;         % the cycle period
% end

%% constrains of u_rho, u_T

clear_t=zeros(1,8);
% set a parameter k (A larger value of k yields shorter cycles, while a smaller value of k yields longer cycles. )
global kk 
kk = 0.5;

% node 1
[u_T(5),u_T(6),u_T(8),u_T(4)]=deal(60);
[u_rho(5),u_rho(6),u_rho(8),u_rho(4)]=deal(0.5);

% node 2
[u_T(1),u_T(2),u_T(3),u_T(11),u_T(12),u_T(15),]=deal(60);
[u_rho(1),u_rho(2),u_rho(3),u_rho(12)]=deal(0.5);
u_rho(15)=deal(0.2);
u_rho(11)=deal(0.2);
clear_t(2) = (1-(u_rho(1)+u_rho(15)+u_rho(11)))*u_T(1)/3;
S_0(1,11)=u_rho(15)*u_T(15)+2*clear_t(2);
S_0(1,15)=clear_t(2);

% node 3
[u_T(7),u_T(9),u_T(10),u_T(18),u_T(19)]=deal(60);
[u_rho(18),u_rho(19)]=deal(0.5);
[u_rho(10),u_rho(9)]=deal(0.3);
u_rho(7)=deal(0.1);
clear_t(3) = (1-(u_rho(7)+u_rho(18)+u_rho(9)))*u_T(7)/3;
S_0(1,7)=u_rho(9)*u_T(9)+2*clear_t(3);
[S_0(1,9),S_0(1,10)]=deal(clear_t(3));

% node 4
[u_T(13),u_T(21)]=deal(60);
[u_rho(13),u_rho(21)]=deal(0.5);

% node 5
[u_T(16),u_T(17),u_T(22),u_T(23),u_T(27),u_T(28),]=deal(40);
[u_rho(16),u_rho(17)]=deal(0.3);
u_rho(28)=deal(0.1);
[u_rho(22),u_rho(23)]=deal(0.3);
u_rho(27)=deal(0.1);
clear_t(5) = (1-(u_rho(16)+u_rho(28)+u_rho(22)+u_rho(27)))*u_T(16)/4;
S_0(1,27)=u_rho(28)*u_T(28)+2*clear_t(5);
[S_0(1,22),S_0(1,23)]=deal(u_rho(27)*u_T(27)+S_0(1,27)+clear_t(5));
S_0(1,28)=clear_t(5);

% node 6
[u_T(20),u_T(24),u_T(25)]=deal(60);
[u_rho(24),u_rho(25)]=deal(0.45);
u_rho(20)=deal(0.45);
clear_t(6) = (1-(u_rho(20)+u_rho(24)))*u_T(20)/2;
S_0(1,20)=clear_t(6);

% node 7
[u_T(14),u_T(26),u_T(31)]=deal(60);
[u_rho(14)]=deal(0.3);
[u_rho(31)]=deal(0.3);
u_rho(26)=deal(0.3);
clear_t(7) = (1-(u_rho(14)+u_rho(31)+u_rho(26)))*u_T(14)/3;
S_0(1,26)=u_rho(31)*u_T(31)+2*clear_t(7);
S_0(1,31)=clear_t(7);

% node 8
[u_T(29),u_T(30)]=deal(60);
[u_rho(29),u_rho(30)]=deal(0.5);



% u_rho=zeros(1,N);
% u_rho(1:N)=1;
% u_T=zeros(1,N);
% u_T(1:N)=1;
% u_rho(17)=0.03;
% u_T(17)=500;



for i=1:N
    X0=[X0;S_0(:,i)]  ;
end

%% Pack initial states of every vehicle

S0_v=zeros(n_v,N_v);
for i=1:N_v
        a = iniArc(i);
        S0_v(:,i) = [0;round((rand*(5-1)+1)*10)/10;0;a;unidrnd(Arc{3,a});unidrnd(Arc{3,a});1];
        for j=1:i-1
            if S0_v(4,j)==S0_v(4,i) && S0_v(5,j)==S0_v(5,i)
                S0_v(7,i)=S0_v(7,j)+1;
            end
        end
%       round((rand*(5-1)+1)*10)/10    unidrnd(Arc{3,a})
        % the initial state of vehicles are defined as follow:
        % the clock is set to 0
        % the velocity is a random number between 1 cell/sec and 5 cell/sec
        % the current arc is the initial arcs we calculated before
        % the current lane is random among the lanes of the current arc
        % the target lane is random among the lanes of the current arc
        % the current position is cell 1(if not occupied)
        
        % Compute the optimal time(time without red traffic lights) for each
        % vehicle
        optTime(i)=totalCells(i)/ S0_v(2,i);
end

S0_v(2,1)=3.6;
S0_v(2,2)=3.8;
S0_v(2,3)=4.0;

for i=1:N_v
    X0=[X0;S0_v(:,i)]  ;
end

X0=[X0;0]  ;% Set average queue length to zero

% Last path that the vehicle passed
global lastPath
lastPath = zeros(1,N_v);

%% Define the hybrid system variables

global dt
Tf = 1 * T_c;            % final time (flow time). 
dt = 1e-3;               % sampling resolution for flows
TSPAN = [0 Tf];          % flow time span  
JSPAN = 1e100*TSPAN;         % jump time span 
rule=1;
options = odeset('RelTol',1e-2,'AbsTol',1e-2,'MaxStep',.1);

%% Simulation
[time,jumps,X] =HyEQsolver( @FlowMap,@JumpMap,@Cset,@Dset,X0,TSPAN,JSPAN,rule,options);

%% Plot simulation results
% 
W=X';
% % plot the controller 
figure;
plot(time,W(1,:))
title('the controllers clock')
figure;
plot(time,W(2,:))
axis([-inf inf 0.5 h+0.5])
title('interval of the day (eta)')

% plot each traffic light
% Extract every traffic light of the system ( starts from the 4-th varialbe of X )
Q = W(4:2*N+3,:);
[p,q]=size(Q);
S=zeros(2,q,N);
for i=1:N
    S(:,:,i)=ExtractState(n,N,i,Q);
end

% Extract clocks and traffic light values
value=zeros(1,q,N);
tao=zeros(1,q,N);
for i =1:N
    value(:,:,i)= S(2,:,i);
    tao(:,:,i)= S(1,:,i);
end

% % plot all traffic light values
% figure;
% for i=1:N
% plot(time,value(:,:,i))
% axis([-inf inf -0.5 1.5])
% legend_str{i}=['traffic light' num2str(i)];
% hold on
% end
% legend(legend_str)
% title('traffic light values');

% plot traffic light values of each node
figure;
plot(time,value(:,:,4));hold on
plot(time,value(:,:,5));hold on
plot(time,value(:,:,6));hold on
plot(time,value(:,:,8));hold on
legend('TL#4','TL#5','TL#6','TL#8');
axis([-inf inf -0.5 1.5])
title('traffic light values for #4,5,6,8 in node 1');

figure;
plot(time,value(:,:,1));hold on
plot(time,value(:,:,11));hold on
plot(time,value(:,:,15));hold on
legend('TL#1,2,3,12','TL#11','TL#15');
axis([-inf inf -0.5 1.5])
title('traffic light values for #1,2,3,12,11,15 in node 2');

figure;
plot(time,value(:,:,7));hold on
plot(time,value(:,:,9));hold on
plot(time,value(:,:,18));hold on
legend('TL#7','TL#9,10','TL#18,19');
axis([-inf inf -0.5 1.5])
title('traffic light values for #7,9,10,18,19 in node 3');

figure;
plot(time,value(:,:,13));hold on
plot(time,value(:,:,21));hold on
legend('TL#13','TL#21');
axis([-inf inf -0.5 1.5])
title('traffic light values for #13,21 in node 4');

figure;
plot(time,value(:,:,16));hold on
plot(time,value(:,:,22));hold on
plot(time,value(:,:,27));hold on
plot(time,value(:,:,28));hold on
legend('TL#16,17','TL#22,23','TL#27','TL#28');
axis([-inf inf -0.5 1.5])
title('traffic light values for #16,22,27,28 in node 5');

figure;
plot(time,value(:,:,20));hold on
plot(time,value(:,:,24));hold on
legend('TL#20','TL#24,25');
axis([-inf inf -0.5 1.5])
title('traffic light values for #20,24,25 in node 6');

figure;
plot(time,value(:,:,14));hold on
plot(time,value(:,:,26));hold on
plot(time,value(:,:,31));hold on
legend('TL#14','TL#26','TL#31');
axis([-inf inf -0.5 1.5])
title('traffic light values for #14,26,31 in node 7');

figure;
plot(time,value(:,:,29));hold on
plot(time,value(:,:,30));hold on
legend('TL#29','TL#30');
axis([-inf inf -0.5 1.5])
title('traffic light values for #29.30 in node 8');

% % plot clocks of traffic lights
% figure;
% for i=1:N
% plot(time,tao(:,:,i))
% legend_str{i}=['clock' num2str(i)];
% hold on
% end
% legend(legend_str)
% title('clocks of traffic lights');

% % plot values of traffic lights
% for i=1:N
% figure;
% plot(time,value(:,:,i))
% axis([-inf inf -0.5 1.5])
% title(['Traffic Light ' num2str(i)]);  
% end

% plot average queue length
figure;
plot(time,W(end,:))
title('the average queue length')

% Calculate the delay time for each vehicle
DelayTime=zeros(1,N_v);
for i=1:N_v
    [~,ind] = min(W(2*N+10+7*(i-1),:)); % First index the vehicle is in cell 0
    arriveTime(i) = time(ind);
    DelayTime(i)= arriveTime(i) - optTime(i);
end
m=find(DelayTime<0);
DelayTime(m)=[];
AveDelayTime =mean(DelayTime(:))
MaxDelayTime = max(DelayTime(:))
MaxQueueLength = max(W(end,:))

toc
