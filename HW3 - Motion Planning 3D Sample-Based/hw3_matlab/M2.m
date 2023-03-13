% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples in PRM
%        num_neighbors -> Integer denoting number of closest neighbors to
%                         consider in PRM
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: samples -> num_samples x 4 matrix, sampled configurations in the
%                    roadmap (vertices)
%         adjacency -> num_samples x num_samples matrix, the weighted
%                      adjacency matrix denoting edges between roadmap
%                      vertices. adjacency(i,j) == 0 if there is no edge
%                      between vertex i and j; otherwise, its value is the
%                      weight (distance) between the two vertices. For an
%                      undirected graph, the adjacency matrix should be
%                      symmetric: adjacency(i,j) == adjacency(j,i)

function [samples, adjacency] = M2(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_centers, sphere_radii)
    samples = [];
    adjacency = zeros(num_samples, num_samples);
    while length(samples) < num_samples
        c1 = q_min(1) + (q_max(1)-q_min(1))*rand(1,1);
        c2 = q_min(2) + (q_max(2)-q_min(2))*rand(1,1);
        c3 = q_min(3) + (q_max(3)-q_min(3))*rand(1,1);
        c4 = q_min(4) + (q_max(4)-q_min(4))*rand(1,1);
        random_config = [c1,c2,c3,c4];
        if ~check_collision(robot, random_config, link_radius, sphere_centers, sphere_radii)
            samples(end+1,:) = random_config;
        end
    end
    
    for i=1:length(samples)
        current = samples(i,:);
        Idx = knnsearch(samples, current, 'K',10);
        for j=1:num_neighbors
            neighbor = samples(Idx(j),:);
            if ~check_edge(robot, current, neighbor,link_radius, sphere_centers, sphere_radii, 11)
                distance = pdist([current;neighbor]);
                adjacency(i,Idx) = distance;
                adjacency(Idx,i) = distance;
            end
            
        end
        
    end
    
    
    
end