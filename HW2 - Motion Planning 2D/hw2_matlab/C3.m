% Input: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_goal -> 2x1 vector denoting the goal configuration
% Output: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise

function distances = C3(cspace, q_grid, q_goal)
    x_start = find(abs(q_grid-0.85) < 0.03);
    y_start = find(abs(q_grid-0.90) < 0.03);
    x_goal = find(abs(q_grid-q_goal(1)) < 0.03);
    y_goal = find(abs(q_grid-q_goal(2)) < 0.03);
    open = [x_goal, y_goal];
    cost = cspace;
    cost(x_goal, y_goal) = 2;
    matrix_size = 100;
    adjacent = [ 1 -1; 1 0; 1 1; 0 -1; 0 0; 0 1; -1 -1; -1 0; -1 1 ];
    while size(open,1) ~= 0
      % Iterate through cells adjacent to the cell at the top of the open queue:
      for k=1:size(adjacent,1)
        
        % Calculate index for current adjacent cell:
        adj = open(1,:)+adjacent(k,:);
        % Make sure adjacent cell is in the map
        if min(adj) < 1
          continue
        end
        if adj(1) > matrix_size
          continue
        end
        if adj(2) > matrix_size
          continue
        end
        % Make sure the adjacent cell is not an obstacle 
        if cost(adj(1), adj(2)) == 1
          continue
        end
        % Make sure the adjacent cell is not closed:
        if cost(adj(1), adj(2)) ~= 0
          continue
        end
        % Set the cost and add the adjacent to the open set
        cost(adj(1), adj(2)) = cost(open(1,1), open(1,2)) + 1;
        open(size(open,1)+1,:) = adj;
      end

      % Pop the top open cell from the queue
      open = open(2:end,:);
    end
    
    distances = cost;
end