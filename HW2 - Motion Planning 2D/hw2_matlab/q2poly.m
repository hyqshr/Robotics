% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to convert to polygons
% Output: poly1 -> A polyshape object denoting the 2-D polygon describing
%                  link 1
%         poly2 -> A polyshape object denoting the 2-D polygon describing
%                  link 2
%         pivot1 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 1 (frame 1 origin), with respect to base frame
%         pivot2 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 2 (frame 2 origin), with respect to base frame

function [poly1, poly2, pivot1, pivot2] = q2poly(robot, q)
    q = q*(-1);
    q(2) = q(2)+q(1);
    c1 = cos(q(1));
    s1 = sin(q(1));
    c2 = cos(q(2));
    s2 = sin(q(2));
    
    pivot1 = robot.pivot1;
    pivot2 = [2.1*c1; -2.1*s1];
    link1 = [0.5*s1-1.2*c1 -0.5*s1-1.2*c1 -0.4*s1+2.3*c1 0.4*s1+2.3*c1; 1.2*s1+0.5*c1 1.2*s1-0.5*c1 -2.3*s1-0.4*c1 -2.3*s1+0.4*c1] + robot.pivot1;
    link2 = robot.pivot1 + pivot2 + [0.4*s2-0.3*c2 -0.4*s2-0.3*c2 -0.2*s2+2.7*c2 0.2*s2+2.7*c2; 0.3*s2+0.4*c2 0.3*s2-0.4*c2 -2.7*s2-0.2*c2 -2.7*s2+0.2*c2];
    poly1 = polyshape(link1(1,:), link1(2,:));
    poly2 = polyshape(link2(1,:), link2(2,:));
    pivot2 = robot.pivot1 + pivot2;
    
    
    
    
    
    

end