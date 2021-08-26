function [next_road] = Psi( NewPathArcs, arc, d)

    global Net startNode endNode Nodes2Arc Arc

    %  we compute the next arc index, using current arc and iniPath_arcs
%     CurrentArcLocation = find(NewPathArcs == arc);
    if  endNode(arc)==d   % Vehicle is arriving the destination
        next_road(2) = 0;
        next_road(1) = 0;
    else
        next_road(2) = NewPathArcs(1); % Next arc is equal to the next arc alone the path
        next_road(1) = unidrnd(Arc{3,next_road(2)});       % Next lane is random among the lanes of the next arc
    end

end