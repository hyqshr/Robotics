% Input: q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples to sample
% Output: qs -> num_samples x 4 matrix of joint angles,
%               all within joint limits

function qs = M1(q_min, q_max, num_samples)
    qs = [];
    for i=1:num_samples
        c1 = q_min(1) + (q_max(1)-q_min(1))*rand(1,1);
        c2 = q_min(2) + (q_max(2)-q_min(2))*rand(1,1);
        c3 = q_min(3) + (q_max(3)-q_min(3))*rand(1,1);
        c4 = q_min(4) + (q_max(4)-q_min(4))*rand(1,1);
        random_config = [c1,c2,c3,c4];
        qs(i,:) = random_config;
    end
    
    
end