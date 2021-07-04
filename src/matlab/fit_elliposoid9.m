function[ Center,Scale_axis] = fit_elliposoid9( data )
% input data is n*3,  n points of the ellipsoid surface
% Least Square Method
%  a(1)x^2+a(2)y^2+a(3)z^2+a(4)xy+a(5)xz+a(6)yz+a(7)x+a(8)y+a(9)z=1
% output Center is 1*3, center of elliposoid, 
% Scale_axis is 1*3, 3 axis' scale
% author  Zhang Xin

x=data(:,1);
y=data(:,2);
z=data(:,3);
               
D=[x.*x y.*y z.*z x.*y x.*z y.*z x y z ];
s = D'*D;
k = inv(D'*D);
a=inv(D'*D)*D'*ones(size(x));
M=[a(1) a(4)/2 a(5)/2;...
   a(4)/2 a(2) a(6)/2;...
   a(5)/2 a(6)/2 a(3)]; 
Center=-1/2*[a(7),a(8),a(9)]*inv(M);
SS=Center*M*Center'+1;
[U,V]=eig(M);                       %Matlab计算特征值是大小顺序
[~,n1]=max(abs(U(:,1)));            %输出的，但和xyz轴顺序不   
[~,n2]=max(abs(U(:,2)));            %不同，这个操作就是让特征
[~,n3]=max(abs(U(:,3)));            %值和xyz轴对应上。
lambda(n1)=V(1,1);
lambda(n2)=V(2,2);
lambda(n3)=V(3,3);
Scale_axis=[sqrt(SS/lambda(1)),sqrt(SS/lambda(2)),sqrt(SS/lambda(3))];
% Center=round(Center);
% Scale_axis=round(Scale_axis);

figure
plot3(x,y,z,'b.',Center(1),Center(2),Center(3),'ro');
axis equal
xlabel('X');
ylabel('Y');
zlabel('Z');

end
