% function [x]=RK43d(x0,u,dt,fun)
% global np ndim
% x=zeros(2*np,ndim);
% 
%    p1=feval(fun,x0,u);
%    p2=feval(fun,x0+(dt/2)*p1,u);
%    p3=feval(fun,x0+(dt/2)*p2,u);
%    p4=feval(fun,x0+(dt)*p3,u);
%    x=x0+dt*((1/6)*p1+(1/3)*p2+(1/3)*p3+(1/6)*p4);
%    %x(:,:,k+1)=x(:,:,k)+dt*p1;


