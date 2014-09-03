clc;
clear;
close all;
fileID = fopen('path_matlab_nyq.csv');
C = textscan(fileID,'%f,%f,%f,%f');
fclose(fileID);
x_t=cell2mat(C(2));%theoretical path
y_t=cell2mat(C(3));%theoretical path
z_t=cell2mat(C(4));%theoretical path
fileID = fopen('results_nyq.csv');
C = textscan(fileID,'%f,%f,%f,%f');
fclose(fileID);
x_res=cell2mat(C(2));%resulting path
y_res=cell2mat(C(3));%reuslting path
z_res=cell2mat(C(4));%reuslting path
figure;
plot(x_t,y_t,'-b');
hold on;
plot(x_res,y_res,'-r');
legend('theoretical path','reuslting path');

delta=0.8;%precision=10cm
index_inf_t=1;
index_sup_t=index_inf_t;
index_inf_res=1;
 [x_res,y_res,z_res]=unique_coordinates(x_res,y_res,z_res);
 x_res=x_res';y_res=y_res';z_res=z_res';
% while(x_res(index_inf_res)==x_res(index_inf_res+1))
%     index_inf_res=index_inf_res+1;
% end
index_sup_res=index_inf_res+1;
k=1;
while(index_sup_t<numel(x_t) && index_sup_res<numel(x_res))
   mom_index_res=[];
   while(isempty(mom_index_res) && index_sup_t<numel(x_t))
        index_sup_t=index_sup_t+1;
        distance=sqrt((x_res(index_sup_res:end)-x_t(index_sup_t)).^2.+(y_res(index_sup_res:end)-y_t(index_sup_t)).^2+(z_res(index_sup_res:end)-z_t(index_sup_t)).^2);
        if(min(distance)<delta)
           [m,mom_index_res]=min(distance);
        else
            mom_index_res=[];
        end
   end
   mom_index_res=mom_index_res+index_sup_res-1;
   MSE(k)=getMSE(x_t(index_inf_t:index_sup_t),y_t(index_inf_t:index_sup_t),z_t(index_inf_t:index_sup_t),x_res(index_inf_res:mom_index_res),y_res(index_inf_res:mom_index_res),z_res(index_inf_res:mom_index_res));
%    if(k==10)
        x_t(index_inf_t:index_sup_t)
        y_t(index_inf_t:index_sup_t)
        x_res(index_inf_res:mom_index_res),y_res(index_inf_res:mom_index_res)
%    end
   k=k+1;
   index_inf_res=mom_index_res;
   index_sup_res=index_inf_res+1;
   index_inf_t=index_sup_t;  
end
figure;
plot(1:numel(MSE),sqrt(MSE))