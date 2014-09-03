clc
clear;
close all;
t=0:pi/100:pi;
r=100;
y=r*sin(8*t);x=r*cos(t);z=-15*ones(1,numel(x));
plot(x,y);
t_q=0:pi/1000:pi;
x_q=interp1(t,x,t_q);
y_q=interp1(t,y,t_q);
z_q=interp1(t,z,t_q);
v=1;
T=1+sqrt((circshift(x',1)'-x).^2+(circshift(y',1)'-y).^2+(circshift(z',1)'-z).^2)/v;
for k=2:numel(T)
   T(k)=T(k)+T(k-1)+rand();
end
T_q=interp1(t,T,t_q);
path=[T;x;y;z];
fileID = fopen('path_matlab_def.csv','w');
fprintf(fileID,'%f,%f,%f,%f\n',path);
fclose(fileID);
path_q=[T_q;x_q;y_q;z_q];
fileID = fopen('path_matlab_q.csv','w');
fprintf(fileID,'%f,%f,%f,%f\n',path_q);
fclose(fileID);