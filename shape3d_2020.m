clear all
close all
clc
%function [value,th]=consensus3(gamma0,lambda0)
global np ndim sigma K beta pp flag alpha M zdes
flagcolision=0;
tic
flag=0;
M=50;

ndim=3;
sigma=0;
K=10;
beta=0.1;
alpha=1.1;
alpha1=10;
pp=4; 
np=50; %number of agents

gamma0=5;
lambda0=0.3;
%generate random ic
for i=1:np
    for kk=1:ndim
            x00(i,kk)=2*rand-1;
            v00(i,kk)=2*rand-1;
    end
end
vbar0=1/np*sum(v00,1);
%make initial vbar=0  comment next 3 lines for vbar!=0
v00(:,1)=-vbar0(1)+v00(:,1);
v00(:,2)=-vbar0(2)+v00(:,2);
v00(:,3)=-vbar0(3)+v00(:,3);
vbar2=1/np*sum(v00,1);
Gamma=0;
Lambda=0;
%scaling of ics for having gamma0 and lambda0
for i=1:np
    for j=1:np
        Gamma=Gamma+norm(x00(i,:)-x00(j,:),2).^2;
        Lambda=Lambda+norm(v00(i,:)-v00(j,:),2).^2;
    end
end

Gamma=1/(2*np^2)*Gamma;
Lambda=1/(2*np^2)*Lambda;

x00=sqrt(gamma0/Gamma)*x00;
v00=sqrt(lambda0/Lambda)*v00;



ti=0;
tf=5; %simulation time
dt=0.01; %simulation step
t=ti:dt:tf;
nsteps=length(t);
x=zeros(2*np,ndim,nsteps);
% save('z000.mat','x00','v00')
%load('z000.mat')
x(:,:,1)=[x00;v00];
x0=x00;
v0=v00;

beta2=0;
distance=zeros(np,np);
u=zeros(np,ndim,nsteps-1);
vbar=zeros(1,2);
i=0;
p=0.1;
q=0.9;

Rad=0;
theta=0; %formation acquisition 
% 
% for ii=1:np-1
%     theta=theta+norm(x0(ii,:)-x0(ii+1,:)-zdes(ii,:),2).^2;
% end
dd=NaN; %collision detection
for i=1:np
    for j=1:np
        if i~=j
            dd=min(norm(x0(i,:)-x0(j,:),2).^2,dd);
        end
    end
end

DD(1)=dd;
% Theta(1)=theta;
Gamma(1)=gamma0;
Lambda(1)=lambda0;

VB(1,:)=vbar0;
zdes=zeros(np-1,ndim);

% FORMATION SHAPES
% square shape 4 agents
% zdes=[];
% zdes(1,:)=[0 5];
% zdes(2,:)=[5 0];
% zdes(3,:)=[0 -5];

% any number of agents - square grid shape
%zdes=[];
% ss=2;
% ss1=0.5;
% NN=ceil(sqrt(np));
% kk=1;
% tt=0;
% flagg=1;
% while kk<=np-1 
%     if kk-tt<=NN
%         zdes(kk,:)=flagg*[0 ss -1];
%         kk=kk+1;
%         if kk-tt==NN&&kk~=np
%             zdes(kk,:)=[ss 0 -1];
%             tt=tt+NN;
%             flagg=-flagg;
%             kk=kk+1;
%            
%         end
%     end
% end
 
% any number of agents - regular poligon with agent at the center
% zdes=[];
% R=8;
% NN=np-1;
% theta1=2*pi/(NN);
% phi=2*pi*rand(1)/8;
% zdes(1,:)=-[R*cos(phi) R*sin(phi) 0];
% zdes(2,:)=-[R*cos(phi+theta1) R*sin(phi+theta1) 0]+[R*cos(phi) R*sin(phi) 0];
% 
% for i=3:NN
%     zdes(i,:)=[R*cos(phi+theta1*(i-2)) R*sin(phi+theta1*(i-2)) 0]-[R*cos(phi+theta1*(i-1)) R*sin(phi+theta1*(i-1)) 0]+[0 0 i];
% end
%     
% bird-like flocking
% vbardir=vbar0/norm(vbar0);
% anglevbdir=atan2(vbardir(2),vbardir(1));
% 
% for i=1:floor(np/2)
%     zdes(i,:)=-[2*cos(anglevbdir-10*2*pi/180) 2*sin(anglevbdir-10*2*pi/180)];
% end
% 
% for i=floor(np/2)+1:np-1
%     zdes(i,:)=[2*cos(anglevbdir+10*2*pi/180) 2*sin(anglevbdir+10*2*pi/180)];
% end

%olympic rings 2d
Nr = np/5; % agents per ring
angle = linspace(0,(Nr-1)*2*pi/Nr,Nr);

xb = cos(angle) * 0.9;
yb = sin(angle) * 0.9;

xy = cos(angle+0.1) * 0.9 + 1;
yy = sin(angle+0.1) * 0.9 - 1;

xk = cos(angle+0.2) * 0.9 + 2;
yk = sin(angle+0.2) * 0.9;

xg = cos(angle+0.1) * 0.9 + 3;
yg = sin(angle+0.1) * 0.9 - 1;

xr = cos(angle+0.2) * 0.9 + 4;
yr = sin(angle+0.2) * 0.9;
zdes=[];
zdes(1,:)=-[xb(2) yb(2) 0]+[xb(1) yb(1) 0];

for i=2:np/5-1
    zdes(i,:)=[xb(i) yb(i) 0]-[xb(i+1) yb(i+1) 0];
end
i=np/5;
jj=1;
zdes(i,:)=[xb(i) yb(i) 0]-[xy(jj) yy(jj) 0];
for i=np/5+1:2*np/5-1
    zdes(i,:)=[xy(jj) yy(jj) 0]-[xy(jj+1) yy(jj+1) 0];
    jj=jj+1;
end
i=2*np/5;
zdes(i,:)=[xy(jj) yy(jj) 0]-[xk(1) yk(1) 0];

jj=1;
for i=2*np/5+1:3*np/5-1
    zdes(i,:)=[xk(jj) yk(jj) 0]-[xk(jj+1) yk(jj+1) 0];
    jj=jj+1;
end 
i=3*np/5;
zdes(i,:)=[xk(jj) yk(jj) 0]-[xr(1) yr(1) 0];

jj=1;
for i=3*np/5+1:4*np/5-1
    zdes(i,:)=[xr(jj) yr(jj) 0]-[xr(jj+1) yr(jj+1) 0];
    jj=jj+1;
end 
i=4*np/5;
zdes(i,:)=[xr(jj) yr(jj) 0]-[xg(1) yg(1) 0];

jj=1;
for i=4*np/5+1:np-1
    zdes(i,:)=[xg(jj) yg(jj) 0]-[xg(jj+1) yg(jj+1) 0];
    jj=jj+1;
end




xinitt=[x00(:,1);v00(:,1);x00(:,2);v00(:,2);x00(:,3);v00(:,3)];
[t,X]=ode23s(@csdynmatlab,0:dt:tf,xinitt);
X=X';
ftime=length(t);
x1=X(1:np,:);
v1=X(np+1:2*np,:);
x2=X(2*np+1:3*np,:);
v2=X(3*np+1:4*np,:);
x3=X(4*np+1:5*np,:);
v3=X(5*np+1:6*np,:);


value=Lambda(end);
% th=Theta(end);

%end

figure
col = jet(np);
hold on
for i=1:np
    plot3(x1(i,:),x2(i,:),x3(i,:),'Color',col(i,:))
    plot3(x1(i,1),x2(i,1),x3(i,1),'Color',col(i,:),'Marker','x','MarkerSize',10)
    plot3(x1(i,ftime),x2(i,ftime),x3(i,ftime),'Color',col(i,:),'Marker','square','MarkerSize',10)
end



% for i=1:1
%    plot(squeeze(x(i,1,:)),squeeze(x(i,2,:)),'LineWidth',2.0,'Color','red')
% end
% 
% N=np;
% col = jet(N);
% trajectories with initial and final points highlighted. 
% for kk=1:np
%   comment for removing trajectories
%  plot3(squeeze(x(kk,1,1:termino)),squeeze(x(kk,2,1:termino)),squeeze(x(kk,3,1:termino)),'Color',col(kk,:),'LineWidth',2,'LineStyle','-') 
%   plot3(squeeze(x(kk,1,1)),squeeze(x(kk,2,1)),squeeze(x(kk,3,1)),'Color',[0.5 0.5 1],'Marker','x','MarkerSize',5)
%   plot3(squeeze(x(kk,1,termino)),squeeze(x(kk,2,termino)),squeeze(x(kk,3,termino)),'Color',[sqrt(1-1)/N sqrt(1-1)/N sqrt(1-1)/N],'Marker','square','MarkerSize',10,'MarkerFaceColor',col(kk,:))
% end
% obtain range of trajectories to fix camera for animation
% a=get(gca,'xlim');b=get(gca,'ylim');c=get(gca,'zlim'); 


% for i=1:N
%     plot(z(1,(i-1)*d+1),z(1,(i-1)*d+2),'Color',[sqrt(1-1)/N sqrt(1-1)/N sqrt(1-1)/N],'Marker','o','MarkerSize',10)
% 	plot(z(:,(i-1)*d+1),z(:,(i-1)*d+2),'Color',[sqrt(i-1)/N sqrt(i-1)/N sqrt(i-1)/N],'LineWidth',2,'LineStyle','-');
%     plot(z(length(z),(i-1)*d+1),z(length(z),(i-1)*d+2),'Color',[sqrt(1-1)/N sqrt(1-1)/N sqrt(1-1)/N],'Marker','o','MarkerSize',10)
% end

% figure
% plot([0:dt:tf],Gamma,'LineWidth',2.0)
% title('Gamma')
% 
% figure
% plot([0:dt:tf],Lambda,'LineWidth',2.0)
% title('Lambda')
% figure
% plot([0:dt:tf],Theta,'LineWidth',2.0,'Color',[0.5 0.5 0])
% title('Theta')
% figure
% plot([0:dt:tf],DD,'LineWidth',2.0)
% title('collision detection')

% figure
% plot(VB,'LineWidth',2.0,'Color',[1 0 0])


