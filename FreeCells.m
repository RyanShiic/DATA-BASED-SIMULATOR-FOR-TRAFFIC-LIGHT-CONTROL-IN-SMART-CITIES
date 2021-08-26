function [u_i_E] = FreeCells(lane, arc, alpha_i, l_i, c_i)

global Arc
% this function is to display freecells in road (lane, arc)
% alpha_i, l_i and c_i are the state variables of all vehicles

% compute occupied cells
c_o=[];
for i=1:length(alpha_i)
    if (alpha_i(i) == arc) && (l_i(i) == lane)
        c_o = [c_o , c_i(i)];
    end
end

a = Arc{2,arc};
u_i_E = (1:a);
[~, ia, ~] = intersect(u_i_E,c_o); 
u_i_E(ia) = [];

end