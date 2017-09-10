function [C, R] = LinearPnP(X, x, K)

%% LinearPnP
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 1) pose rotation
%
% IMPORTANT NOTE: While theoretically you can use the x directly when solving
% for the P = [R t] matrix then use the K matrix to correct the error, this is
% more numeically unstable, and thus it is better to calibrate the x values
% before the computation of P then extract R and t directly

O=[0,0,0,0];
A=[];
N=size(x,1);
for i=1:N
    
 u=x(i,1);
 v=x(i,2);

a=[O,-X(i,:),-1,v*X(i,:),v;
    X(i,:),1,O,-u*X(i,:),-u;
    -v*X(i,:),-v,u*X(i,:),u,O];

A=[A;a];


end

[u,d,v]=svd(A);

p=v(:,end);
p=reshape(p,[3,4]);
p1=p(1:3,1:3);
R=inv(K)*p1;

[u1,d1,v1]=svd(R);

t=inv(K)*p(:,4)/d1(1,1);
d1(3,3)=0;
R=u1*d1*v1;
C=-R'*t;
end


