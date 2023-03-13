arr = angle4;
arr = arr * 4 * -1;
%arr = [-1.5, -2.5, -3.5, -4.5, -5.5, -6.5, -7.5, -8.5, -9.5, -10.5, -11.5, -12.5, -13.5, -14.5, -15.5, -16.5, -17.5, -18.5, -19.5, -20.5, -21.5];
len = length(arr);

current = zeros(1, len);
count_state = zeros(1, len);

ma_per_degree = 85;
detent_width = 10;

last_current = 0;
last_angle = 0;
count = 0;
peak_current_threshold = (ma_per_degree / 1000.0) * (detent_width / 2) * 0.8;
threshold = detent_width / 2;
recent_current_positive = false;

for i = 1:len
    curr_angle = arr(i);
    phase = rem(curr_angle, detent_width); % 0...9.999
    
    if curr_angle > 0
        if phase < threshold % positive phase 0...4.999
            if last_current < 0 % - -> + transition ex: 5.2 -> 4.95
                if abs(phase - threshold) > 0.4 % well past edge, switch direction
                    new_current = phase * (ma_per_degree / 1000.0);
                else
                    new_current = last_current;
                end
            else % last_current < 0 ex: 3.4 -> 3.5 
                new_current = phase * (ma_per_degree / 1000.0);
            end
        else % phase > detent_width / 2 % negative current phase 5.0...9.999
            if last_current > 0 % + -> - transition ex 4.95 -> 5.2
                if abs(phase - threshold) > 0.4 % well past edge, switch direction
                    phase2 = rem(phase, threshold);
                    new_current = (threshold - phase2) * (ma_per_degree / 1000.0) * -1;
                else
                    new_current = last_current;
                end
            else % last_current > 0 ex: 5.5 -> 5.6
                phase2 = rem(phase, threshold);
                new_current = (threshold - phase2) * (ma_per_degree / 1000.0) * -1;
            end
        end
    else % negative angle values
        if phase > -threshold % negative phase -4.999...0
            if last_current > 0 % + -> - transition ex: -5.1 -> -4.8
                if abs(phase - -threshold) > 0.4
                    new_current = phase * (ma_per_degree / 1000.0);
                else
                    new_current = last_current;
                end
            else
                new_current = phase * (ma_per_degree / 1000.0);
            end
        else % positive phase -9.999...-5.0
            if last_current < 0 % - -> + transition ex: -4.8 -> -5.1
                if abs(phase - -threshold) > 0.4
                    phase2 = rem(phase, threshold);
                    new_current = (threshold + phase2) * (ma_per_degree / 1000.0);
                else
                    new_current = last_current;
                end
            else
                phase2 = rem(phase, threshold);
                new_current = (threshold + phase2) * (ma_per_degree / 1000.0);
            end
        end
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