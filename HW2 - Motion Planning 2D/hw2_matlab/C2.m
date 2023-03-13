% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
% Output: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise

function cspace = C2(robot, obstacles, q_grid)
    cspace=zeros(100, 100);
        for i = 1:100
            for j = 1:100
                for n = 1 : length(obstacles)
                    q = [q_grid(i), q_grid(j)];
                    [poly1, poly2, ~, ~] = q2poly(robot, q);
                    polyout1 = intersect(poly1,obstacles(n)).NumRegions;
                    polyout2 = intersect(poly2,obstacles(n)).NumRegions;
                    if ((polyout1 == 1)) || ((polyout2 == 1))
                        cspace(i,j) = 1;
                    end
                
                end
            end
        end
end