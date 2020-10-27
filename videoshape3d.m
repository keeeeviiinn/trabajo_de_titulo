vidObj = VideoWriter('triang2.avi');
open(vidObj);
writerObj.Height=1080;
writerObj.Width=1920;


angle = linspace(0,(100-1)*2*pi/100,100);

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



    % Create an animation.

figure('units','pixels','position',[0 0 1920 1080])
    set(gca,'nextplot','replacechildren');
   
    axis([a b c])
    view(-19,48) %fix perspective 
    grid on
     N=np;
for k=1:10

for j=1:np
   plot3(squeeze(x(j,1,1)),squeeze(x(j,2,1)),squeeze(x(j,3,1)),'Color',[0.5 0.5 1],'Marker','o','MarkerSize',12,'MarkerFaceColor',col(j,:))
   hold on
end
% plot3(xr, yr,zeros(length(yr)),'Color','r','LineWidth',2,'LineStyle','--')
% plot3(xb,yb,zeros(length(yr)),'Color','b','LineWidth',2,'LineStyle','--')
% plot3(xg,yg,zeros(length(yr)),'Color','g','LineWidth',2,'LineStyle','--')
% plot3(xy,yy,zeros(length(yr)),'Color','y','LineWidth',2,'LineStyle','--')
% plot3(xk,yk,zeros(length(yr)),'Color','k','LineWidth',2,'LineStyle','--')

   

hold off
   axis([a b c])
    view(-19,48)
     grid on

  currFrame = getframe(gcf);
  writeVideo(vidObj,currFrame);
end

% 
for k=1:1:200

hold off
   for j=1:np
   plot3(squeeze(x(j,1,1:k)),squeeze(x(j,2,1:k)),squeeze(x(j,3,1:k)),'Color',col(j,:),'LineWidth',2,'LineStyle','-') 
  hold on
   plot3(squeeze(x(j,1,k)),squeeze(x(j,2,k)),squeeze(x(j,3,k)),'Color',[0.5 0.5 1],'Marker','o','MarkerSize',12,'MarkerFaceColor',col(j,:))
   end
% plot3(xr, yr,zeros(length(yr)),'Color','r','LineWidth',2,'LineStyle','--')
% plot3(xb,yb,zeros(length(yr)),'Color','b','LineWidth',2,'LineStyle','--')
% plot3(xg,yg,zeros(length(yr)),'Color','g','LineWidth',2,'LineStyle','--')
% plot3(xy,yy,zeros(length(yr)),'Color','y','LineWidth',2,'LineStyle','--')
% plot3(xk,yk,zeros(length(yr)),'Color','k','LineWidth',2,'LineStyle','--')

   


   axis([a b c])
    view(-19,48)
     grid on


  currFrame = getframe(gcf);
  writeVideo(vidObj,currFrame);
end

ei=48;%stop and pan camera
for ii=1:80-48


for j=1:np
   plot3(squeeze(x(j,1,1:200)),squeeze(x(j,2,1:200)),squeeze(x(j,3,1:200)),'Color',col(j,:),'LineWidth',2,'LineStyle','-') 
   hold on
   plot3(squeeze(x(j,1,200)),squeeze(x(j,2,200)),squeeze(x(j,3,200)),'Color',[0.5 0.5 1],'Marker','o','MarkerSize',12,'MarkerFaceColor',col(j,:))
  
end
% plot3(xr, yr,zeros(length(yr)),'Color','r','LineWidth',2,'LineStyle','--')
% plot3(xb,yb,zeros(length(yr)),'Color','b','LineWidth',2,'LineStyle','--')
% plot3(xg,yg,zeros(length(yr)),'Color','g','LineWidth',2,'LineStyle','--')
% plot3(xy,yy,zeros(length(yr)),'Color','y','LineWidth',2,'LineStyle','--')
% plot3(xk,yk,zeros(length(yr)),'Color','k','LineWidth',2,'LineStyle','--')
hold off
   
   ei=ei+1;
   
   axis([a b c])
   view(-19,ei)
  grid on

  currFrame = getframe(gcf,[0 0 1920 1080]);
  writeVideo(vidObj,currFrame);
end

%continue
for k=201:5:1000

   for j=1:np
   plot3(squeeze(x(j,1,k-200:k)),squeeze(x(j,2,k-200:k)),squeeze(x(j,3,k-200:k)),'Color',col(j,:),'LineWidth',2,'LineStyle','-') 
     hold on
   plot3(squeeze(x(j,1,k)),squeeze(x(j,2,k)),squeeze(x(j,3,k)),'Color',[0.5 0.5 1],'Marker','o','MarkerSize',12,'MarkerFaceColor',col(j,:))

   end
% plot3(xr, yr,zeros(length(yr)),'Color','r','LineWidth',2,'LineStyle','--')
% plot3(xb,yb,zeros(length(yr)),'Color','b','LineWidth',2,'LineStyle','--')
% plot3(xg,yg,zeros(length(yr)),'Color','g','LineWidth',2,'LineStyle','--')
% plot3(xy,yy,zeros(length(yr)),'Color','y','LineWidth',2,'LineStyle','--')
% plot3(xk,yk,zeros(length(yr)),'Color','k','LineWidth',2,'LineStyle','--')
% 
%    
   hold off


   axis([a b c])
    view(-19,80)
     grid on

  currFrame = getframe(gcf);
  writeVideo(vidObj,currFrame);
end


for k=1001:20:5000

   for j=1:np
     plot3(squeeze(x(j,1,800:k)),squeeze(x(j,2,800:k)),squeeze(x(j,3,800:k)),'Color',col(j,:),'LineWidth',2,'LineStyle','-') 
     hold on
   plot3(squeeze(x(j,1,k)),squeeze(x(j,2,k)),squeeze(x(j,3,k)),'Color',[0.5 0.5 1],'Marker','o','MarkerSize',12,'MarkerFaceColor',col(j,:))
   end
%     plot3(xr, yr,zeros(length(yr)),'Color','r','LineWidth',2,'LineStyle','--')
% plot3(xb,yb,zeros(length(yr)),'Color','b','LineWidth',2,'LineStyle','--')
% plot3(xg,yg,zeros(length(yr)),'Color','g','LineWidth',2,'LineStyle','--')
% plot3(xy,yy,zeros(length(yr)),'Color','y','LineWidth',2,'LineStyle','--')
% plot3(xk,yk,zeros(length(yr)),'Color','k','LineWidth',2,'LineStyle','--')

     hold off

   axis([a b c])
   ei=80+(k-1001)*0.1;
   if ei<90 
    view(-19,ei)
   else
    view(-19,90)
   end
     grid on



 
  currFrame = getframe(gcf);
  writeVideo(vidObj,currFrame);
end

for k=5001:500:termino/2

   for j=1:np
   plot3(squeeze(x(j,1,5001:k)),squeeze(x(j,2,5001:k)),squeeze(x(j,3,5001:k)),'Color',col(j,:),'LineWidth',2,'LineStyle','-') 
     hold on
   plot3(squeeze(x(j,1,k)),squeeze(x(j,2,k)),squeeze(x(j,3,k)),'Color',[0.5 0.5 1],'Marker','o','MarkerSize',12,'MarkerFaceColor',col(j,:))

   end
%     plot3(xr, yr,zeros(length(yr)),'Color','r','LineWidth',2,'LineStyle','--')
% plot3(xb,yb,zeros(length(yr)),'Color','b','LineWidth',2,'LineStyle','--')
% plot3(xg,yg,zeros(length(yr)),'Color','g','LineWidth',2,'LineStyle','--')
% plot3(xy,yy,zeros(length(yr)),'Color','y','LineWidth',2,'LineStyle','--')
% plot3(xk,yk,zeros(length(yr)),'Color','k','LineWidth',2,'LineStyle','--')

   
   hold off

   axis([a b c])
   aa=-19+200*(k-5001)*19*0.01/termino;
   if aa<0 
       view(aa,90)
   else
       view(0,90)
   end
   	
  
   

   %view(0,90)
     grid on
  currFrame = getframe(gcf);
  writeVideo(vidObj,currFrame);
end

for k=termino/2:600:termino*3/3

   for j=1:np
   plot3(squeeze(x(j,1,5001+(k-termino/2):k)),squeeze(x(j,2,5001+(k-termino/2):k)),squeeze(x(j,3,5001+(k-termino/2):k)),'Color',col(j,:),'LineWidth',2,'LineStyle','-') 
     hold on
   plot3(squeeze(x(j,1,k)),squeeze(x(j,2,k)),squeeze(x(j,3,k)),'Color',[0.5 0.5 1],'Marker','o','MarkerSize',12,'MarkerFaceColor',col(j,:))

   end
%     plot3(xr, yr,zeros(length(yr)),'Color','r','LineWidth',2,'LineStyle','--')
% plot3(xb,yb,zeros(length(yr)),'Color','b','LineWidth',2,'LineStyle','--')
% plot3(xg,yg,zeros(length(yr)),'Color','g','LineWidth',2,'LineStyle','--')
% plot3(xy,yy,zeros(length(yr)),'Color','y','LineWidth',2,'LineStyle','--')
% plot3(xk,yk,zeros(length(yr)),'Color','k','LineWidth',2,'LineStyle','--')

   
   hold off

   axis([a b c])
   aa=-19+200*(k-5001)*19*0.01/termino;
   if aa<0 
       view(aa,90)
   else
       view(0,90)
   end
   	
  
   

   %view(0,90)
     grid on
  currFrame = getframe(gcf);
  writeVideo(vidObj,currFrame);
end



    % Close the file.
    close(vidObj);