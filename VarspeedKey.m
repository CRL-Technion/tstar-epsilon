% cells = [1 0 0; 1 1 0; 0 1 0; -1 1 0; -1 0 0; 1 1 1;...
%     0 1 1; -1 1 1; -1 0 1; -1 -1 1];
% cells = [1 0; 1 1; 0 1; -1 1; -1 0; -1 -1; 0 -1; 1 -1];
cells = [1 0; 1 1; 0 1; -1 1; -1 0; -1 -1; 0 -1; 1 -1];
neighbors = 7; % 0-7 nighbors id
speeds = 1; % 0-1 speed
orient = 7; % 0-7 orientations
path_id = 34; % 34 options for path id
type = 5;
num_of_keys = 1;
positions = [];
keys = [];
for curr_speed = 0:speeds
   for curr_orient = 0: orient
      for target_speed = 0: speeds
         for target_orient = 0:orient
             for target_neighbor = 0:neighbors
                 for path = 1:path_id
                    key = curr_speed+speeds*(curr_orient+orient*(target_speed+speeds*...
                        (target_orient+orient*(target_neighbor+neighbors*path))));
                    if find(keys == key)
                        plot = ' key alreadt exist ';
                        X = ['iteration: ', num2str(num_of_keys),num2str(key), plot, 'cell ', num2str(find(keys == key))];
                        disp(X)
                    return
                    end
                    keys(num_of_keys) = key;
    %             positions(num_of_keys) = [cells(i,1) cells(i,2) cells(i,3) j k];
                    num_of_keys = num_of_keys + 1;
                 end
             end
         end
      end
   end
end

a = find(keys == -1);