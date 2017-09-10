function [proj_points, t, R] = ar_cube(H,render_points,K)
%% ar_cube
% Estimate your position and orientation with respect to a set of 4 points on the ground
% Inputs:
%    H - the computed homography from the corners in the image
%    render_points - size (N x 3) matrix of world points to project
%    K - size (3 x 3) calibration matrix for the camera
% Outputs: 
%    proj_points - size (N x 2) matrix of the projected points in pixel
%      coordinates
%    t - size (3 x 1) vector of the translation of the transformation
%    R - size (3 x 3) matrix of the rotation of the transformation
% Written by Stephen Phillips for the Coursera Robotics:Perception course

% YOUR CODE HERE: Extract the pose from the homography
if H(3,3)<0
    H=-H;
end
h1=H(:,1);
h2=H(:,2);
h3=H(:,3);

R_=[h1,h2,cross(h1,h2)];

[U,D,V]=svd(R_);

N=eye(3);
N(3,3)=det(U*V');

R= U*N*V';
R;
t=h3/norm(h1);
t;

% YOUR CODE HERE: Project the points using the pose
proj_points=[];
for xj=1:8
XP=K*(R*(render_points(xj,:)')+t);
proj_points=[proj_points;XP(1)/XP(3),XP(2)/XP(3)];
end

end
