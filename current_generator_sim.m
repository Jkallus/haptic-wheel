arr = angle4;
arr = arr * 4;
len = length(arr);

current = zeros(1, len);
count_state = zeros(1, len);

ma_per_degree = 85;
detent_width = 10;

last_current = 0;
last_angle = 0;
count = 0;
peak_current_threshold = (ma_per_degree / 1000.0) * (detent_width / 2) * 0.8;
recent_current_positive = false;

for i = 1:len
    curr_angle = arr(i);
    phase = rem(curr_angle, detent_width); % 0...9.999
    
    if phase < detent_width / 2 % positive phase 0...4.999
        if last_current < 0 % - -> + transition ex: 5.2 -> 4.95
            if abs(phase - detent_width / 2) > 0.4 % well past edge, switch direction
                new_current = phase * (ma_per_degree / 1000.0);
            else
                new_current = last_current;
            end
        else % last_current < 0 ex: 3.4 -> 3.5 
            new_current = phase * (ma_per_degree / 1000.0);
        end
    else % phase > detent_width / 2 % negative current phase 5.0...9.999
        if last_current > 0 % + -> - transition ex 4.95 -> 5.2
            if abs(phase - detent_width / 2) > 0.4 % well past edge, switch direction
                phase2 = rem(phase, detent_width / 2);
                new_current = ((detent_width / 2) - phase2) * (ma_per_degree / 1000.0) * -1;
            else
                new_current = last_current;
            end
        else % last_current > 0 ex: 5.5 -> 5.6
            phase2 = rem(phase, detent_width / 2);
            new_current = ((detent_width / 2) - phase2) * (ma_per_degree / 1000.0) * -1;
        end
    end
    
    if i == 10513
        x = 0;
    end
    
    at_snap_edge = abs(last_current) > peak_current_threshold & abs(new_current) > peak_current_threshold;
    if(last_current > 0) && (new_current < 0) && at_snap_edge
        count = count - 1;
    elseif (last_current < 0) && (new_current > 0) && at_snap_edge
        count = count + 1;  
    end

    last_current = new_current;
    last_angle = curr_angle;
    current(i) = new_current;
    count_state(i) = count;
end     
    %end
%end

%start_idx = 10510;
%end_idx = 10550;

start_idx = 1;
end_idx = len;

close all;
figure;
subplot(2,1,1);
yyaxis left;
plot(current(start_idx:end_idx));
ylabel('Current');
yyaxis right;
plot(count_state(start_idx:end_idx));
ylabel('Count');
subplot(2,1,2);
plot(arr(start_idx:end_idx));
ylabel('Degrees');