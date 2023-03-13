% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_path -> Mx2 matrix containing a collision-free path from
%                  q_start to q_goal. Each row in q_path is a robot
%                  configuration. The first row should be q_start,
%                  the final row should be q_goal.
% Output: num_collisions -> Number of swept-volume collisions encountered
%                           between consecutive configurations in q_path

function num_collisions = C6(robot, obstacles, q_path)

    num_collisions = 0;
    for i=2:size(q_path,1)
        config_x = q_path(i,1);
        config_y = q_path(i,2);
        last_config_x = q_path(i-1,1);
        last_config_y = q_path(i-1,2);
        config = [config_x, config_y];
        last_config = [last_config_x, last_config_y];
        [~, last_poly2, ~, ~] = q2poly(robot,last_config);
        [~, poly2, ~, ~] = q2poly(robot,config);
        polyinit = union(last_poly2,poly2);
        polyout = convhull(polyinit);
        for n = 1 : length(obstacles)
            if intersect(polyout,obstacles(n)).NumRegions >= 1
                disp("collision detected! config:")
                disp([config_x, config_y])
                num_collisions = num_collisions + 1;
            end
        end
        
    end
        
end

