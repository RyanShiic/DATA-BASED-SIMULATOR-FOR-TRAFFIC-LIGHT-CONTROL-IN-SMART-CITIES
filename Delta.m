function [target_lane]=Delta(lane, arc, next_road, cell )
    global Lambada Arc

    NumLane = Arc{3,arc}; % Number of lanes of in the current arc

    % We check if the current road can go to the "next road"
    [Lia, ~] = ismember(next_road,Lambada(lane,arc), 'rows');
    if Lia==1  % The vehicle dose not need to change lane (including the exiting road)
        target_lane = lane;
    else       % The vehicle needs to change to a larger or smaller lane, or the vehicle is arriving the destination
        if next_road(2)==0 % The vehicle is arriving the destination
            target_lane = lane;
        else               % The vehicle needs to change to a larger or smaller lane
            for i=1:NumLane
                [lia, ~] = ismember(next_road,Lambada(i,arc), 'rows');
                if lia==1
                    target_lane = i;
                    break
                end
            end
        end
    end
    %  target_lane = lane;
end