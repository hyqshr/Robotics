% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found

function [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    sample_nodes = 500;
    nodes = [q_start];
    G = graph();
    path=[];
    bias = 0.08;
    beta = 0.1;
    % sample 
    while length(nodes) < sample_nodes
        % random config
        if rand() > beta
            c1 = q_min(1) + (q_max(1)-q_min(1))*rand(1,1);
            c2 = q_min(2) + (q_max(2)-q_min(2))*rand(1,1);
            c3 = q_min(3) + (q_max(3)-q_min(3))*rand(1,1);
            c4 = q_min(4) + (q_max(4)-q_min(4))*rand(1,1);
            random_config = [c1,c2,c3,c4];
        % low chance we use the goal config as our target config
        else 
            random_config = q_goal;
            c1 = q_goal(1);
            c2 = q_goal(2);
            c3 = q_goal(3);
            c4 = q_goal(4);
        end
        Idx_closest = knnsearch(nodes, random_config);
        closest_neighbor = nodes(Idx_closest,:);
        new_node = closest_neighbor + (random_config - closest_neighbor) * (repmat(bias,1,4)/abs(random_config - closest_neighbor));
        
        % boundary collision check
        in_bounds = (all(new_node >= q_min) && all(new_node <= q_max));
        if check_edge(robot, closest_neighbor, new_node, link_radius, sphere_centers, sphere_radii + 0.01, 100) || ~in_bounds 
            continue
        end
        nodes(end+1,:) = new_node;
        G = addedge(G, Idx_closest, length(nodes));
    end
    
    % find the shorest path
    Idx_goal = knnsearch(nodes, q_goal);
    [path_row,len] = shortestpath(G,1,Idx_goal);
    
    % if no find path
    if len == inf
        disp("Path not found!! ")
        path_found= false;
        return
    end
    
    % if find a path
    path_found= true;
    for i=1:length(path_row)
        path(end+1,:) = nodes(path_row(i),:);
    end
    
end