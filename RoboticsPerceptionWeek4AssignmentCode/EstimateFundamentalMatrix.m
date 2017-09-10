function F = EstimateFundamentalMatrix(x1, x2)
%% EstimateFundamentalMatrix
% Estimate the fundamental matrix from two image point correspondences 
% Inputs:
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Output:
%    F - size (3 x 3) fundamental matrix with rank 2

A=[];
N=size(x1,1);
for i=1:N
    u1=x1(i,1);
    u2=x2(i,1);
    v1=x1(i,2);
    v2=x2(i,2);
    temp=[u1*u2,u1*v2,u1,v1*u2,v1*v2,v1,u2,v2,1];
    A=[A;temp];
end

[u,d,v]=svd(A);

x=v(:,9);

F=reshape(x,[3,3]);

[u,d,v]=svd(F);

d(3,3)=0;

F=u*d*v';
end