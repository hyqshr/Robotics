% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

function smoothed_path = M5(robot, path, link_radius, sphere_centers, sphere_radii)
    % create another graph to optimize the path by checking if one can reach each other 
    G = graph();
    for i=1:length(path)
        for j=i:length(path)
            
            if ~check_edge(robot, path(i,:), path(j,:), link_radius, sphere_centers, sphere_radii, 50)
                G = addedge(G, i, j);
            end
            
        end
    end
    [path_row,len] = shortestpath(G,1,length(path));
    smoothed_path = path(path_row,:);
end