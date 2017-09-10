function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE

N=[];
for i=1:4
    x1=video_pts(i,1);
    y1=video_pts(i,2);
    x1_=logo_pts(i,1);
    y1_=logo_pts(i,2);
p1=[-x1,-y1,-1,0,0,0,x1*x1_,y1*x1_,x1_;0,0,0,-x1,-y1,-1,x1*y1_,y1*y1_,y1_];
%p2=[-x2,-y2,-2,0,0,0,x2*x2_,y2*x2_,x2_;0,0,0,-x2,-y2,-2,x2*y2_,y2*y2_,y2_];
%p3=[-x3,-y3,-3,0,0,0,x3*x3_,y3*x3_,x3_;0,0,0,-x3,-y3,-3,x3*y3_,y3*y3_,y3_];
%p4=[-x4,-y4,-4,0,0,0,x4*x4_,y4*x4_,x4_;0,0,0,-x4,-y4,-4,x4*y4_,y4*y4_,y4_];
N=[N;p1];
end

[U, S, V] = svd(N);

l=size(V,2);

h=V(:,l);
H = [h(1) h(2) h(3);h(4) h(5) h(6);h(7) h(8) h(9)];

end

