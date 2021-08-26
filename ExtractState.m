function [State] = ExtractState(n,N,i,X)

% This function is to extract i-th state variables from a packed state
% space. For example: a packed state space: X=[tao_1; value_1;tao_2; value_2;tao_3; value_3;tao_4; value_4]
% ExtractState(2,X)= [tao_2; value_2]
% Where n is the number of state variales of one sub-state space, In this
% example n = 2; N is the total number of sub-state space, In this
% example N = 4

    
    [p,q]=size(X);

    S=zeros(n,q,N);
    for k=1:N
        S(:,:,k)= X((1+(k-1)*n):k*n,:);
    end
    State =  S(:,:,i);
end
    