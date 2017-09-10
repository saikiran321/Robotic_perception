function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     K - size (3 x 3) camera calibration (intrinsics) matrix
%     x
% Outputs: 
%     X - size (N x 3) matrix of refined point 3D locations

[m,n]=size(X0);

X=zeros(m,3);

for i=1:m
    
    np=Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1(i,:), x2(i,:), x3(i,:), X0(i,:));
    np1=Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1(i,:), x2(i,:), x3(i,:), np');
    np2=Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1(i,:), x2(i,:), x3(i,:), np1');
 
end

end

function X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
J = [Jacobian_Triangulation(C1, R1, K, X0);
         Jacobian_Triangulation(C2, R2, K, X0);
         Jacobian_Triangulation(C3, R3, K, X0);];
   
X=X0';
f=K(1,1);
px=K(1,3);
py=K(2,3);
R={R1,R2,R3};
pp={};
pp{1}=K*R1*(X-C1);
pp{2}=K*R2*(X-C2);
pp{3}=K*R3*(X-C3);
u={};
v={};
w={};
for i=1:3
    du_dx=[f*R{1}(1,1)+px*R{i}(3,1),f*R{i}(1,2)+px*R{i}(3,2),f*R{i}(1,3)+px*R{i}(3,3)];
    dv_dx=[f*R{i}(2,1)+py*R{i}(3,1),f*R{i}(2,2)+py*R{i}(3,2),f*R{i}(2,3)+py*R{i}(3,3)];
    dw_dx=[R{i}(3,1),R{i}(3,2),R{i}(3,3)];
    ii=pp{1};
    ui=ii(1);
    vi=ii(2);
    wi=ii(3);
    u{i}=ii(1);
    v{i}=ii(2);
    w{i}=ii(3); 
end

b=[x1(1) x1(2) x2(1) x2(2) x3(1) x3(2)]';
fx=[u{1}/w{1} v{1}/w{1} u{2}/w{2} v{2}/w{2} u{3}/w{3} v{3}/w{3}]';


del_x= inv(J.'*J)*(J.')*(b-fx);

X=X+del_x;


end

function J = Jacobian_Triangulation(C, R, K, X)
    X=X';
    X
    x = K*R*(X-C);
    f=K(1,1);
    p_x = K(1,3);
    p_y = K(2,3);
    
    dev_w_X = R(3,:); 
    dev_u_X = [f*R(1,1)+p_x*R(3,1) f*R(1,2)+p_x*R(3,2) f*R(1,3)+p_x*R(3,3)];
    dev_v_X = [f*R(2,1)+p_y*R(3,1) f*R(2,2)+p_y*R(3,2) f*R(2,3)+p_x*R(3,3)];
    J = [ (x(3)*dev_u_X-x(1)*dev_w_X)/(x(3)*x(3)) ; (x(3)*dev_v_X-x(2)*dev_w_X)/(x(3)*x(3)) ];
end


