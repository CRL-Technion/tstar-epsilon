% cells = [1 0 0; 1 1 0; 0 1 0; -1 1 0; -1 0 0; 1 1 1;...
%     0 1 1; -1 1 1; -1 0 1; -1 -1 1];
cells = [1 0; 1 1; 0 1; -1 1; -1 0; -1 -1; 0 -1; 1 -1];
orient = 7;
type = 5;
num_of_keys = 1;
positions = [];
keys = [];
for i = 1:length(cells)
    for startOrient = 0:orient
        for endOrinet = 0:orient
            for k = 0:type
                key = cells(i,1)*3+5*(cells(i,2)+7*(endOrinet+11*(k+13 + 17*startOrient)));
                if find(keys == key)
                    plot = ' key alreadt exist ';
                    X = [num2str(key), plot, 'cell ', num2str(i)];
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
a = find(keys == -1);