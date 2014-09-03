function distance=distanceVector(x_t,y_t,z_t,x_res,y_res,z_res)
    
%     t=0:1/(numel(x_t)-1):1;
%     t_q=0:1/(numel(x_res)-1):1;
%     x_q=interp1(t,x_t,t_q);
%     y_q=interp1(t,y_t,t_q);
%     z_q=interp1(t,z_t,t_q);
%     distance=(x_res'-x_q).^2.+(y_res'-y_q).^2+(z_res'-z_q).^2;
% end

    t=0:1/(numel(x_t)-1):1;
    t_q=0:1/(1000*(numel(x_res)-1)):1;
    x_q=interp1(t,x_t,t_q);
    y_q=interp1(t,y_t,t_q);
    z_q=interp1(t,z_t,t_q);
    distance=zeros(1,numel(x_res));
    for k=1:numel(x_res)
        [~,pos]=min(abs(x_q-x_res(k)));
        distance(k)=sqrt((x_q(pos)-x_res(k)).^2.+(y_q(pos)-y_res(k)).^2+(z_q(pos)-z_res(k)).^2);
    end
end
