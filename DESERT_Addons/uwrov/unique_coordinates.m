function [x,y,z]=unique_coordinates(x_res,y_res,z_res)
x(1)=x_res(1);
y(1)=y_res(1);
z(1)=z_res(1);
index=2;
for k=2:numel(x_res)
    if( x_res(k)==x(index-1) && y_res(k)==y(index-1) && z_res(k)==z(index-1))
        
    else
       x(index)= x_res(k);
       y(index)= y_res(k);
       z(index)= z_res(k);
       index=index+1;
    end
end
end