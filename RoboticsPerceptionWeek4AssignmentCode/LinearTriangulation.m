function X = LinearTriangulation(K, C1, R1, C2, R2, x1, x2)
%% LinearTriangulation
% Find 3D positions of the point correspondences using the relative
% position of one camera from another
% Inputs:
%     C1 - size (3 x 1) translation of the first camera pose
%     R1 - size (3 x 1) rotation of the first camera pose
%     C2 - size (3 x 1) translation of the second camera
%     R2 - size (3 x 1) rotation of the second camera pose
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Outputs: 
%     X - size (N x 3) matrix whos rows represent the 3D triangulated
%       points

P1=K*R1*[eye(3),-C1];
P2=K*R2*[eye(3),-C2];

X=[];
N=size(x1,1);

for i=1:N
    xp=[x1(i,1);x1(i,2);1];
    v1=Vec2Skew(xp)*P1;
    xp2=[x2(i,1);x2(i,2);1];
    v2=Vec2Skew(xp2)*P2;
    T=[v1;v2];
    [~,~,V]=svd(T);
    nV=V(1:3,end)/V(end,end);
    X=[X;nV'];
end

end