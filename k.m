function [next_cell]=k(c_i, F)
 if ismember(c_i+1, F)
     next_cell = c_i+1;
 else
     next_cell = c_i;
 end
end