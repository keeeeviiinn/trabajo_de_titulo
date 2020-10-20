function [x]=csdynshape(xinit,u)
global np ndim beta pp flag alpha zdes M K 

flag2=0;
if u==1
    flag2=1;
end
u=zeros(np,ndim);

a=zeros(np,np,ndim);
f=zeros(np,np,ndim);
distance=zeros(np,np);
difv=zeros(np,np,ndim);
difx=zeros(np,np,ndim);
Np=np;
x0=xinit(1:np,:);
v0=xinit(np+1:2*np,:);
mdiag=-eye(np);
front=[[zeros(1,np-1);eye(np-1)] zeros(np,1)];
back=[zeros(np,1) [eye(np-1);zeros(1,np-1)]];

for i=1:np
    distance(i,:)=sqrt(sum((repmat(x0(i,:),Np,1)-x0).^2,2));
    difv(i,:,:)=v0-repmat(v0(i,:),Np,1);
    difx(i,:,:)=-(x0-repmat(x0(i,:),Np,1));
end
distform=[];
for i=1:np-1
    %distance from desired formation
    distform(i)=norm(x0(i,:)-x0(i+1,:)-zdes(i,:),2);
end

distance=distance+eye(length(distance)); %this is to avoid division by zero
alignment=1*sqrt((0.5/np)*(sum(sum(difv(:,:,1).^2))+sum(sum(difv(:,:,2).^2))));
for k=1:ndim;
    a(:,:,k)=K*influence(distance,alpha).*difv(:,:,k);
    f(:,:,k)=repulsion(distance,pp).*difx(:,:,k);
end

u(1,:)=-M*influenceu(distform(1),beta)*(x0(1,:)-x0(2,:)-zdes(1,:));
u(Np,:)=M*influenceu(distform(Np-1),beta)*(x0(Np-1,:)-x0(Np,:)-zdes(Np-1,:));
for i=2:Np-1
    u(i,:)=M*influenceu(distform(i-1),beta)*(x0(i-1,:)-x0(i,:)-zdes(i-1,:))-M*influenceu(distform(i),beta)*(x0(i,:)-x0(i+1,:)-zdes(i,:));
end

%zdes=[z1 z2];
%     temp=diag(distform,1);
%     temp2=diag(distform,-1);
%     distform=distform-diag(temp,1)+diag(temp2,1);
%     for k=1:ndim;
%         au(:,:,k)=influenceu(distform,beta).*(-difx(:,:,k)+(back-front)*zdes(:,k)).*(back+front);
%     end
%     u=squeeze(sum(au,2));

% (1/np)*
if flag2
    u=u+[50 0;zeros(np-1,2)];
end
x(1:np,:)=v0;
x(np+1:2*np,:)=squeeze(sum(a,2))./np+0*alignment*squeeze(sum(f,2))+1*u;
end