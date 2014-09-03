function MSE=getMSE(x_t,y_t,z_t,x_res,y_res,z_res)
    t=0:1/(numel(x_t)-1):1;
    t_q=0:1/(1000*(numel(x_res)-1)):1;
    x_q=interp1(t,x_t,t_q);
    y_q=interp1(t,y_t,t_q);
    z_q=interp1(t,z_t,t_q);
    error=0;
    for k=1:numel(x_res)
        
        [~,pos]=min(abs(x_q-x_res(k)));
        error=error+(x_q(pos)-x_res(k)).^2.+(y_q(pos)-y_res(k)).^2+(z_q(pos)-z_res(k)).^2;
        %error=error+min((x_q-x_res(k)).^2.+(y_q-y_res(k)).^2+(z_q-z_res(k)).^2);
    end
    MSE=error/numel(x_res);
end

% function MSE=getMSE(x_t,y_t,z_t,x_res,y_res,z_res)
%     t=0:1/(numel(x_t)-1):1;
%     t_q=0:1/(numel(x_res)-1):1;
%     x_q=interp1(t,x_t,t_q);
%     y_q=interp1(t,y_t,t_q);
%     z_q=interp1(t,z_t,t_q);
%     error=(x_res'-x_q).^2.+(y_res'-y_q).^2+(z_res'-z_q).^2;
%     MSE=sum(error)/numel(error);
% end