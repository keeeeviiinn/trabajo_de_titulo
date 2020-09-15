vidObj = VideoWriter('video.avi');
open(vidObj);
writerObj.Height=1080;
writerObj.Width=1920;
    % Create an animation.

figure('units','pixels','position',[0 0 1080 1080])
    set(gca,'nextplot','replacechildren');
   
    axis([a b])
     %fix perspective 
    grid on
     N=np;
for k=1:10
hold off
for j=1:np
   plot(x1(j,1),x2(j,1),'Color',[0.5 0.5 1],'Marker','o','MarkerSize',12,'MarkerFaceColor',col(j,:))
   hold on
   quiver(x1(j,1),x2(j,1),v1(j,1),v2(j,1),'Color',col(j,:),'LineWidth',2)
% quiver(x,y,u,v) muestra los vectores de velocidad como flechas con los componentes (u,v) en los puntos (x,y)   
end
% plot3(xr, yr,zeros(length(yr)),'Color','r','LineWidth',2,'LineStyle','--')
% plot3(xb,yb,zeros(length(yr)),'Color','b','LineWidth',2,'LineStyle','--')
% plot3(xg,yg,zeros(length(yr)),'Color','g','LineWidth',2,'LineStyle','--')
% plot3(xy,yy,zeros(length(yr)),'Color','y','LineWidth',2,'LineStyle','--')
% plot3(xk,yk,zeros(length(yr)),'Color','k','LineWidth',2,'LineStyle','--')

hold on
   axis([a b])
     grid on
 currFrame = getframe(gcf);
 writeVideo(vidObj,currFrame);
end


for k=1:floor(0.05/dt):nsteps

hold off
   for j=1:np
   plot(x1(j,1:k),x2(j,1:k),'Color',col(j,:),'LineWidth',2,'LineStyle','-') 
  hold on
   plot(x1(j,k),x2(j,k),'Color',[0.5 0.5 1],'Marker','o','MarkerSize',12,'MarkerFaceColor',col(j,:))
   quiver(x1(j,k),x2(j,k),v1(j,k),v2(j,k),'Color',col(j,:),'LineWidth',1,'LineStyle','--')

   end

  

   axis([a b])
     grid off
   
    xlim([x1(1,k)-0.2 x1(1,k)+0.2])
    ylim([x2(1,k)-0.2 x2(1,k)+0.2])
    
 currFrame = getframe(gcf);
  writeVideo(vidObj,currFrame);
end

for  j= 5:-0.1:0
    xlim([x1(1,nsteps)-j x1(1,nsteps)+j])
    ylim([x2(1,nsteps)-j x2(1,nsteps)+j])
   
  currFrame = getframe(gcf);
  writeVideo(vidObj,currFrame)
end

    % Close the file.
    close(vidObj);
