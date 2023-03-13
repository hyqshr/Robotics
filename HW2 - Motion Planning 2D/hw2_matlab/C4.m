% Input: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_start -> 2x1 vector denoting the start configuration
% Output: path -> Mx2 matrix containing a collision-free path from q_start
%                 to q_goal (as computed in C3, embedded in distances).
%                 The entries of path should be grid cell indices, i.e.,
%                 integers between 1 and N. The first row should be the
%                 grid cell containing q_start, the final row should be
%                 the grid cell containing q_goal.

function path = C4(distances, q_grid, q_start)
    x_start = find(abs(q_grid-0.85) < 0.03);
    y_start = find(abs(q_grid-0.90) < 0.03);
    x_goal = find(abs(q_grid-3.05) < 0.03);
    y_goal = find(abs(q_grid-0.05) < 0.03);
    distances(x_goal,y_goal) = 2;
    adjacent = [ 1 -1; 1 0; 1 1; 0 -1; 0 0; 0 1; -1 -1; -1 0; -1 1 ];
    path = [x_start; y_start];
    while 1
        i = size(path,2);
        if distances(path(1,i), path(2,i)) == 2
            disp("goal finded!");
            % reverse the array     
            path = path.';
            return
        end
        curc = distances(path(1,i),path(2,i));
        for k=1:size(adjacent,1)
            % Calculate index for current adjacent cell:
            adj = path(:,i)+adjacent(k,:)';
            % Make sure adjacent cell is in the map
            if min(adj) < 2
                continue
            end
            if adj(1) > 100
                continue
            end
            if adj(2) > 100
                continue
            end

            % If this adjacent cell reduces cost, add it to the path.
            if (distances(adj(1),adj(2)) == 0) || (distances(adj(1),adj(2)) == 1)
                continue;
            end
            if distances(adj(1),adj(2)) < curc
                path(:,i+1) = adj;
                break
            end
        end
    end




