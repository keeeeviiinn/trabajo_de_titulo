function varargout = Slider(varargin)
% SLIDER MATLAB code for Slider.fig
%      SLIDER, by itself, creates a new SLIDER or raises the existing
%      singleton*.
% holaaa mundo
%      H = SLIDER returns the handle to a new SLIDER or the handle to
%      the existing singleton*.
%
%      SLIDER('CALLBACK',hObject,eve0ntData,handles,...) calls the local
%      function named CALLBACK in SLIDER.M with the given input arguments.
%
%      SLIDER('Property','Value',...) creates a new SLIDER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Slider_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Slider_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".k
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Slider

% Last Modified by GUIDE v2.5 27-Nov-2020 11:50:52

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Slider_OpeningFcn, ...
                   'gui_OutputFcn',  @Slider_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Slider is made visible.
function Slider_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Slider (see VARARGIN)

% Choose default command line output for Slider
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Slider wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Slider_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;





% --- Executes during object creation, after setting all properties.
function T_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);

% set(handles.T,'Min',0,'Max',1,'Value',0,'SliderStep',[0.01 0.1])
% tiempo=get(handles.T,'Value')
% tiempo

end


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)

%%%%%%%%%%%%%%%%%%%AQUI COMIENZA EL CODIGO%%%%%%%%%%%%%%%%%%%%%

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
 
 global np ndim tf sigma K beta pp flag alpha M zdes cuadrado bandada poligon elegir
 global paredes circulo triangulo piramide olympic dt 
 % reestablece nombre boton stop

set(handles.pushbutton16, 'BackgroundColor' , 'yellow' );
set(handles.stop,'String',"Stop")



    %%Codigo para ingresar cantidad de agentes 
dos = get(handles.radiobutton26,'Value');
tres= get(handles.radiobutton27,'Value');
if dos ==1;


a=get(gca,'xlim');b=get(gca,'ylim');c=get(gca,'zlim');
axis([a b])
a=a+[-2 2];b=b+[-2 2];  
set (handles.pushbutton16, 'BackgroundColor' , 'green' )    
    
       
set(handles.slider2,'MIN',0,'MAX',20000);
set(handles.slider7,'MIN',0,'MAX',10000);
set(handles.slider10,'MIN',1,'MAX',100);
set(handles.ejey,'MIN',-100,'MAX',100);
set(handles.ejex,'MIN',-100,'MAX',100);
    
npp    = get(handles.edit1,'String');%Se guarda el vaor que se ingrese en el cuadro de texto en guide en la variable "npp"
nppp   = str2double(npp); %el valor de npp se convierte en un numer
if nppp <= 1
    set(handles.text8,'String','Ingrese valor mayor a 1')
elseif isnan (nppp)
    set(handles.text8,'String','Ingrese valor numerico')


else
% set(handles.text8,'String',' ')
%%%%%%%
% close all
% clc
%function [value,th]=consensus3(gamma0,lambda0)
set(handles.text8,'String','')
set(handles.radiobutton8,'Value',0)
set(handles.radiobutton11,'Value',0)
set(handles.slider10,'Value',50)
set(handles.pushbutton5,'String','Play')
set(handles.pushbutton6,'String','Zoom Personal')
set(handles.ejey,'Value',0)
set(handles.ejex,'Value',0)
set(handles.pushbutton9,'visible','on')
movimiento = 0;%Movimiento automatico de los agentes mediante boton

% datos = get(handles,uitable3,'Data');
% datos(:,:) = [];
% set(handles.uitable3,'Data',datos)


flagcolision=0;
tic
original= get(handles.original,'Value');
deseado= get(handles.deseado,'Value');


if original == 1
    
flag=0;
M=50;
%sigma=0 singular influence
sigma=0;
K=100;
beta=0.4;
alpha=1.1;
tf=20;
pp=2; %potencia a la que esta elevada
% np=5; %number of agents
dt=0.001;

end
%OPCION DE PERSONALIZADO 
if deseado == 1
  recibido=get(handles.uitable10,'data');
  
  % recibido= get(handles.uitable10,'data');
  caca = recibido(11,2)
%   ecuacion = 
  x=2;
  y = [];
  formula = inline(caca);
  
  for i=2
      y=[y formula(x(i))]
  end
  
  
  
%valores de la tabla
  v1 = recibido(1,2);
  alpha = str2double(v1);
  v2 = recibido(2,2);
  beta = str2double(v2);
  v3 = recibido(3,2);
  sigma = str2double(v3);
  v4 = recibido(4,2);
  K = str2double(v4);
  v5 = recibido(5,2);
  pp = str2double(v5);
  v6 = recibido(6,2);
  flag = str2double(v6);
  v7 = recibido(7,2);
  M = str2double(v7);
  v7 = recibido(8,2);
  tf = str2double(v7);
  v8 = recibido(9,2);
  dt = str2double(v8)
  
  
end


ndim= 2;
np=nppp;
alpha1=10;
ti=0; 
t=ti:dt:tf;
nsteps=length(t);

gamma0=14;
lambda0=42.3;

x00=zeros(np,ndim);
v00=zeros(np,ndim);

for i=1:np
    for kk=1:ndim
        x00(i,kk)=10*rand-1;
        v00(i,kk)=2*rand-1;
    end
end
vbar0=1/np*sum(v00,1);
vbar2=1/np*sum(v00,1);
zdes=zeros(np-1,ndim);

%%%% en caso de elegir la primera opcion de popmenu %%%

if elegir == 1;
    set(handles.text8,'String','Elegir forma de agente')
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if triangulo == 1
    
    val1=0;
    val2=5;
    val3 = floor(np/2);
    val4= vbar2(2), vbar2(1);
    
end
%%%%%%% CIRCULO2D %%%%%%%%%%%%%%%
if circulo == 1
conta=1;
contador=0;
zdess = 0;
uno = 3;%1
cero5 = 1;%0.5
dif = 0;
while conta < np
    if contador == 0
%         zdes(conta,:) = [uno 0 -cero5];
        zdes(conta,:) = [uno -cero5];
        contador=+1;
        conta=conta+1;
        
    elseif contador == 1
%         zdes(conta,:) = [cero5 0 -uno];
        zdes(conta,:) = [cero5 -uno];
        conta=conta+1;
        contador=contador+1;
    elseif contador == 2
%         zdes(conta,:) = [-cero5 0 -uno];
        zdes(conta,:) = [-cero5 -uno];
        conta=conta+1;
        contador=contador+1; 
    elseif contador == 3
%         zdes(conta,:) = [-uno 0 -cero5];
        zdes(conta,:) = [-uno -cero5];
        conta=conta+1;
        contador=contador+1;
    elseif contador == 4
%         zdes(conta,:) = [-uno 0 cero5];
        zdes(conta,:) = [-uno cero5];
        conta=conta+1;
        contador=contador+1;
    elseif contador == 5
%         zdes(conta,:) = [-cero5 0 uno];
        zdes(conta,:) = [-cero5 uno];
        conta=conta+1;
        contador=contador+1;
    elseif contador == 6
%         zdes(conta,:) = [cero5 0 uno];
        zdes(conta,:) = [cero5 uno];
        conta=conta+1;
        contador=contador+1;
    else
        zdess=+1;    
        zdes(conta,:) = [ -3 0 uno];
%         zdes(conta,:) = [ -3 uno];
        conta=conta+1;
        contador = 0;
        zdess = 0;
        dif = dif +-3;
    end 
end
end

%%%%%%% bird-like flocking %%%%%%%%%%%%%%%
if bandada == 1;
vbardir=vbar0/norm(vbar0);
anglevbdir=atan2(vbardir(2),vbardir(1));

for i=1:floor(np/2)
    zdes(i,:)=-[2*cos(anglevbdir-10*2*pi/180) 2*sin(anglevbdir-10*2*pi/180)];
end

for i=floor(np/2)+1:np-1
    zdes(i,:)=[2*cos(anglevbdir+10*2*pi/180) 2*sin(anglevbdir+10*2*pi/180)];
end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%% any number of agents - regular poligon with agent at the center%%%

if poligon == 1;
zdes=[];
R=4;
NN=np-1;
theta1=2*pi/(NN);
phi=2*pi*rand(1)/8;
zdes(1,:)=-[R*cos(phi) R*sin(phi)];
zdes(2,:)=-[R*cos(phi+theta1) R*sin(phi+theta1)]+[R*cos(phi) R*sin(phi)];

for i=3:NN
    zdes(i,:)=[R*cos(phi+theta1*(i-2)) R*sin(phi+theta1*(i-2))]-[R*cos(phi+theta1*(i-1)) R*sin(phi+theta1*(i-1))];
end
end
%%%%% any number of agents - square grid shape%%%%%%%%
if cuadrado == 1; 
zdes=[];
ss=2;
ss1=0.5;
NN=ceil(sqrt(np));
kk=1;
tt=0;
flagg=1;
while kk<=np-1
    if kk-tt<=NN
        zdes(kk,:)=flagg*[0 ss];
        kk=kk+1;
        if kk-tt==NN&&kk~=np
            zdes(kk,:)=[ss 0];
            tt=tt+NN;
            flagg=-flagg;
            kk=kk+1;

        end
    end
end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initial energies and gamma lambda
Gamma=0;
Lambda=0;
Ec0=0;
for i=1:np
    Ec0=Ec0+norm(v00(i,:),2).^2;
    for j=1:np
        Gamma=Gamma+norm(x00(i,:)-x00(j,:),2).^2;
        Lambda=Lambda+norm(v00(i,:)-v00(j,:),2).^2;
    end
end

Gamma=1/(2*np^2)*Gamma;
Lambda=1/(2*np^2)*Lambda;
%scaling to choose gamma lambda at t=0
x00=sqrt(gamma0/Gamma)*x00;
v00=sqrt(lambda0/Lambda)*v00;
Gammap=0;
Lambdap=0;
Ec0p=0;
D0=0;
for i=1:np
    Ec0p=Ec0p+norm(v00(i,:),2).^2; %kinetic
    for j=1:np
        Gammap=Gammap+norm(x00(i,:)-x00(j,:),2).^2;
        Lambdap=Lambdap+norm(v00(i,:)-v00(j,:),2).^2;
        D0=D0+0.5*(K/np)*influence(norm(x00(i,:)-x00(j,:),2),alpha)*norm(v00(i,:)-v00(j,:),2)^2;
    end
end
Ep0=0;
for i=2:np
    t=norm(x00(i-1,:)-x00(i,:)-zdes(i-1,:),2).^2;
    Ep0=Ep0+M*(sqrt(t+1)-sqrt(1)); %potential for beta=0.5
end

Gammap=1/(2*np^2)*Gammap;
Lambdap=1/(2*np^2)*Lambdap;

% % SQUARE
% np=5;
% % np=nppp;
% modv=1;
% side=2;

% % SQUARE 2
% x00(1,:)=[-side 0]; v00(1,:)=-modv*[1 0];
% x00(2,:)=[0 side]; v00(2,:)=-modv*[-1. 0];
% x00(3,:)=[side 0]; v00(3,:)=-modv*[0 -1];
% x00(4,:)=[0 -side]; v00(4,:)=-modv*[0 1];
% 
% zdes(1,:)=[side side];
% zdes(2,:)=[side -side];
% zdes(3,:)=[-side -side];

x=zeros(2*np,ndim,nsteps);
x(:,:,1)=[x00;v00];
x0=x00;
v0=v00;

beta2=0 ;
distance=zeros(np,np);
u=zeros(np,ndim,nsteps-1);
vbar=zeros(1,2);
i=0;
p=0.1;
q=0.9;
%typeofcontrol=5;
Rad=0;
theta=0;
dd=NaN;
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

        %energy
        Ec=zeros(1,length(0:dt:tf));
        Ep=zeros(1,length(0:dt:tf));
        Ec(1)=Ec0p/2;
        Ep(1)=Ep0;
        DD=zeros(1,length(0:dt:tf));
        DD(1)=D0;
        xinitt=[x00(:,1);v00(:,1);x00(:,2);v00(:,2)];
        [t,X]=ode23s(@csdynmatlab,0:dt:tf,xinitt); % resuelve el sistema de ecuaciones diferenciales con el solver "ode23"
        X=X';
        ftime=length(t);
        x1=X(1:np,:); 
        x2=X(2*np+1:3*np,:);
        v1=X(np+1:2*np,:); % desde np + 1 hasta 2np tenemos la velocidad en el ejex
        v2=X(3*np+1:4*np,:);

        col = jet(1);
        hold on
        d=1;
        ip=1;
        ipp=1;
        dd=0;
        sss=0;

        n = 1;
        movim = 1;
        ss = 1;
        zz = 1;
        ejexx=1;
        ejeyy=1;
      
while   dd==0

    stop=get(handles.stop,'String');
    if stop == "Stop."
       dd=1;
       continue
    end
            
    
    centrar = get(handles.pushbutton10,'String');
    movimiento = get(handles.pushbutton5,'String');
    
    
% if movim == 20001
%     movimiento = "no";
% end

if movim == 20001
    movimiento = "no";
    set(handles.pushbutton5,"String","Reiniciar?");
    Reiniciar = get(handles.pushbutton5,'String');
    while Reiniciar == "Reiniciar?"
    Reiniciar = get(handles.pushbutton5,'String');  
    pause(0.01)
    end
    
    set(handles.slider2,'Value',1)
    movim = 1;
    
    Reinicia = get(handles.pushbutton5,'String'); 
    while Reinicia == "Play"
    Reinicia = get(handles.pushbutton5,'String');  
    pause(0.01)
    end
    
end

%%% AQUI COMIENZA CODIGO PARA GENERAR MOVIMIENTO AUTOMATICO DE LOS AGENTES
if movimiento == "Ok!"
    set(handles.slider2,'Value',movim)
    movim = movim + 200;
    pause(0.1)
end    

        s= get(handles.slider2,'Value');
        s2=get(handles.slider7,'Value');
        dd= get(handles.radiobutton8,'Value');%Guardar valor del boton pausa
        ss=round(s);%ss es la variable que se genera con el slider y se redondea
        ss2=round(s2);
        
        
if      ss==0;
        ss=1;
        
end

if      ss2==0;
        ss2=1;
end

if  sss~=ss | sss2~=ss2 | zz ~= get(handles.slider10,'Value') | ejexx ~= get(handles.ejex,'Value') | ejeyy ~= get(handles.ejey,'Value') | centrar == "Centrado" 
%    if  d~=1
%       if  ss~=sss  
%           ip=sss; 
%     
%       end
%         
% if      ss2~=sss2 
%         ip=sss2;
%     
% end
%     end
        cla%borra lo que se genera anteriormente de los plot, y asi no se sobre escriben y se optimiza
        hold on
        if centrar == "Centrado" % Realiza el centrado de los slider ejex y ejey
            set(handles.ejex,'Value',0)
            set(handles.ejey,'Value',0)
            set(handles.slider10,'Value',50)
            set(handles.pushbutton10,'String',"Centrar")
        end
        
        
         N=np;
         col=jet(N);
        
for     i=1:np
        
          ejey = get(handles.ejey,'Value');
%           ejey = ejey);
          ejex = get(handles.ejex,'Value');
%           ejex = ejex;
          
          
        
        plot(x1(i,ss2:1:ss),x2(i,ss2:1:ss),'LineWidth',2,'LineStyle','-')
%         plot(x1(i,ss),x2(i,ss),'.','MarkerSize',20)
         plot(x1(i,ss),x2(i,ss),'Marker','o','MarkerSize',12,'MarkerFaceColor',col(i,:))
        plot(x1(i,ss2),x2(i,ss2),'Marker','o','MarkerSize',12,'MarkerFaceColor',col(i,:))
        
        
       
        z = get(handles.slider10,'Value');
        zoom_agente = get(handles.pushbutton6,'String');
        
        
        o = get(handles.popupmenu3,'Value');% recibe el valor del menu de agente
        front = get(handles.radiobutton11,'Value');
        back = get(handles.radiobutton12,'Value');
        
        if zoom_agente == "Zoom Personal" & front == 1%% controla la cabeza del agente para realizar zoom de seguimiento
            
            xlim([x1(o,ss)-z+ejex x1(o,ss)+z+ejex])
            ylim([x2(o,ss)-z+ejey x2(o,ss)+z+ejey])%realiza el zoom de acercamiento al zoom en el menu desplegable
            plot(x1(o,ss),x2(o,ss),'*','MarkerSize',12)
            
           
            
        elseif zoom_agente == "Zoom Personal" & back == 1
            xlim([x1(o,ss2)-z+ejex x1(o,ss2)+z+ejex])
            ylim([x2(o,ss2)-z+ejey x2(o,ss2)+z+ejey])
            plot(x1(o,ss2),x2(o,ss2),'*','MarkerSize',12)
         
          
        else
            axis([-z+ejex z+ejex -z+ejey z+ejey]) %realiza el zoom en vista general mediante el slider
       
        end
        sss=ss;
        sss2=ss2;
        d=0;
        zz = get(handles.slider10,'Value');%% Estas variables son para realizar movimiento de zoom y ejes x e y de manera dinamica
        ejexx = get(handles.ejex,'Value');
        ejeyy = get(handles.ejey,'Value');
end
end
        pause(0.01)
        
% Menu desplegable para zoom en un agente en especial
menu = [1:np]; 
    set(handles.popupmenu3,'string',menu)
end

end
cla reset
end


%******** COMIENZO CODIGO 3D*************
if tres == 1


set(handles.salir,'visible','off');
set(handles.salir,'value',1);
set(handles.cabeza,'visible','on');
set(handles.cabeza,'String','Con cabeza');
set(handles.cabezano,'String','Sin cabeza');
set(handles.slider2,'MIN',0,'MAX',101);
set(handles.slider7,'MIN',0,'MAX',101);
set(handles.slider10,'MIN',1,'MAX',100);
set(handles.ejey,'MIN',-100,'MAX',100);
set(handles.ejex,'MIN',-100,'MAX',100); 
set(handles.slider15,'Value',-38);
set(handles.slider16,'Value',29);



npp    = get(handles.edit1,'String');%Se guarda el vaor que se ingrese en el cuadro de texto en guide en la variable "npp"
nppp   = str2double(npp); %el valor de npp se convierte en un numer
% if nppp <= 9
%     set(handles.text8,'String','Ingrese valor > o = a 10 agentes')
% elseif isnan (nppp)
%     set(handles.text8,'String','Ingrese valor numerico')
% 
% 
% else

% set(handles.text8,'String',' ')
%%%%%%%
% close all
% clc
%function [value,th]=consensus3(gamma0,lambda0)
set(handles.text8,'String','')
set(handles.radiobutton8,'Value',0)
set(handles.radiobutton11,'Value',0)
set(handles.slider10,'Value',50)
set(handles.pushbutton5,'String','Play')
set(handles.pushbutton6,'String','Zoom Personal')
set(handles.ejey,'Value',0)
set(handles.ejex,'Value',0)
set(handles.pushbutton9,'visible','on')
set(handles.slider15,'Visible','on');
set(handles.slider16,'Visible','on');
set(handles.text21,'Visible','on');
movimiento = 0;%Movimiento automatico de los agentes mediante boton
 

flagcolision=0;
tic
original= get(handles.original,'Value');
deseado= get(handles.deseado,'Value');
if original == 1
flag=0;
M=50;
tf=400; %simulation time

sigma=0;
K=10;
beta=0.1;
alpha=1.1;
alpha1=10;
pp=4;
dt=0.05;
end
%OPCION DE PERSONALIZADO 
if deseado == 1
  recibido=get(handles.uitable10,'data');
  
  
%valores de la tabla
  v1 = recibido(1,2);
  alpha = str2double(v1)
  v2 = recibido(2,2);
  beta = str2double(v2);
  v3 = recibido(3,2);
  sigma = str2double(v3);
  v4 = recibido(4,2);
  K = str2double(v4);
  v5 = recibido(5,2);
  pp = str2double(v5);
  v6 = recibido(6,2);
  flag = str2double(v6);
  v7 = recibido(7,2);
  M = str2double(v7);
  v7 = recibido(8,2);
  tf = str2double(v7);
  v8 = recibido(9,2);
  dt = str2double(v8)
end

np=nppp;%number of agents
ndim=3;
ti=0;
 %simulation step
t=ti:dt:tf;
nsteps=length(t);
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
% zdes(4,:)=[0 5];
% zdes(5,:)=[5 0];
% zdes(6,:)=[0 -5];
% zdes(7,:)=[0 -5];

% any number of agents - square grid shape
if cuadrado == 1
zdes=[];
ss=2;
ss1=0.5;
NN=ceil(sqrt(np)); % ceil redondea al numero entero mas cercano 
kk=1;
tt=0;
flagg=1;
while kk<=np-1 
    if kk-tt<=NN
        zdes(kk,:)=flagg*[0 ss -1];
        kk=kk+1;
        if kk-tt==NN&&kk~=np
            zdes(kk,:)=[ss 0 -2];
            tt=tt+NN;
            flagg=-flagg;
            kk=kk+1;
           
        end
    end
end
end 
% any number of agents - regular poligon with agent at the center
if poligon == 1;
zdes=[];
R=8;
NN=np-1;
theta1=2*pi/(NN);
phi=2*pi*rand(1)/8;
zdes(1,:)=-[R*cos(phi) R*sin(phi) 0];
zdes(2,:)=-[R*cos(phi+theta1) R*sin(phi+theta1) 0]+[R*cos(phi) R*sin(phi) 0];

for i=3:NN
    zdes(i,:)=[R*cos(phi+theta1*(i-2)) R*sin(phi+theta1*(i-2)) 0]-[R*cos(phi+theta1*(i-1)) R*sin(phi+theta1*(i-1)) 0]+[0 0 i];
end
end
    
% % bird-like flocking
if bandada==1
vbardir=vbar0/norm(vbar0);
anglevbdir=atan2(vbardir(2),vbardir(1));

for i=1:floor(np/2)
    zdes(i,:)=-[2*cos(anglevbdir-10*2*pi/180) 2*sin(anglevbdir-10*2*pi/180) 0];
end

for i=floor(np/2)+1:np-1
    zdes(i,:)=[2*cos(anglevbdir+10*2*pi/180) 2*sin(anglevbdir+10*2*pi/180) 0];
end
end
%olympic rings 2d
if olympic == 1
Nr = np/5; % agents per ring
Nr = np/5;
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
end
%cuadrado
% zdes = [0 2 -1;0 2 -1;2 0 -2; 0 -2 1]; 
% cubo
% zdes = [-2 0 0;0 -2 0;2 0 0; 0 2 -3;-2 0 0;0 -2 0;2 0 0];
% zdes=[];

if piramide == 1
% zdes = [0 2 -1;0 2 -1;2 0 -2; 0 -2 1]; 
%zdes = [6 0 0;3 -6 0]%triangulo
 zdes = [-3 0 0; 3 -3 0;-3 0 0;1.5 1.5 -5;0.5 0.5 2]

 
% cubo
%zdes = [-2 0 0;0 -2 0;2 0 0; 0 2 -3;-2 0 0;0 -2 0;2 0 0];
% zdes=[];

end
%%Paredes
if paredes == 1
conta=1;
contador=0;
zdess = 0;
while conta < np
    if contador == 0
        zdes(conta,:) = [-2 0 zdess];
        contador=+1;
        conta=conta+1;
    elseif contador == 1
        zdes(conta,:) = [0 -2 zdess];
        conta=conta+1;
        contador=contador+1;
    elseif contador == 2
        zdes(conta,:) = [2 0 zdess];
        conta=conta+1;
        contador=contador+1; 
    else
        zdess=+1;    
        zdes(conta,:) = [0 2 zdess];
        conta=conta+1;
        contador = 0;
        zdess = 0;
    end      
pause(0.01)    
end
end
%%circulo
% zdes = [1 0 -0.5 ; 0.5 0 -1 ; -0.5 0 -1 ; -1 0 -0.5  ;-1 0 0.5 ; -0.5 0 1 ; 0.5 0 1];

%%ciculos uno la lado del otro
if circulo == 1
conta=1;
contador=0;
zdess = 0;
uno = 1;
cero5 = 0.5;
dif = 0;
while conta < np
    if contador == 0
        zdes(conta,:) = [uno 0 -cero5];
        contador=+1;
        conta=conta+1;
    elseif contador == 1
        zdes(conta,:) = [cero5 0 -uno];
        conta=conta+1;
        contador=contador+1;
    elseif contador == 2
        zdes(conta,:) = [-cero5 0 -uno];
        conta=conta+1;
        contador=contador+1; 
    elseif contador == 3
        zdes(conta,:) = [-uno 0 -cero5];
        conta=conta+1;
        contador=contador+1;
    elseif contador == 4
        zdes(conta,:) = [-uno 0 cero5];
        conta=conta+1;
        contador=contador+1;
    elseif contador == 5
        zdes(conta,:) = [-cero5 0 uno];
        conta=conta+1;
        contador=contador+1;
    elseif contador == 6
        zdes(conta,:) = [cero5 0 uno];
        conta=conta+1;
        contador=contador+1;
    else
        zdess=+1;    
        zdes(conta,:) = [ -3 0 uno];
        conta=conta+1;
        contador = 0;
        zdess = 0;
        dif = dif +-3;
    end      
pause(0.01)    
end
 end
%%
%RK
for i=1:nsteps-1
    %x(:,:,i+1)=RK4(x(:,:,i),u(:,:,i),dt,@cscdyn);
    x(:,:,i+1)=RK4(x(:,:,i),u(:,:,i),dt,@csdynshape3d);
    x0=squeeze(x(1:np,:,i+1));
    v0=squeeze(x(np+1:end,:,i+1));
    
%     for i=1:np
%     distform(i,:)=sqrt(sum((repmat(x0(i,:),np,1)-x0-repmat(zdes,np,1)).^2,2));
%     end
    gamma=0;
    lambda=0;
    theta=0;
    dd=NaN;
    vbbb=0;
    %lambda gamma theta and dd
    for ii=1:np
        for jj=1:np
            gamma=gamma+norm(x0(ii,:)-x0(jj,:),2).^2;
            lambda=lambda+norm(v0(ii,:)-v0(jj,:),2).^2;
        end
        if ii<np
            %theta=theta+norm(x0(ii,:)-x0(ii+1,:)-zdes,2).^2;
        end
        if ii~=jj
            dd=min(norm(x0(ii,:)-x0(jj,:),2).^2,dd);
        end
        
    end
   % Theta(i+1)=theta/(np-1);
    Gamma(i+1)=1/(2*np^2)*gamma;
    Lambda(i+1)=1/(2*np^2)*lambda;
    %????
    if dd<=1e-20 
        flagcolision=1;
    end
    DD(i+1)=dd;
    VB(i+1,:)=1/np*sum(v0,1);
    % tests to stop if consensus is likely or unlikely to be achieved (for
    % bench)
%     if i>=501 
%         test=sum(Lambda(i-150:i));
%         test2=sum(Theta(i-150:i));
%         if test<=1||test2>=1000
%             break
%             toc
%         end
%     end    
    
end
toc

value=Lambda(end);
% th=Theta(end);

%end
flagcolision;
%figure
axes(handles.axes1)
%hold on
tf=dt*i;
termino=i;
% for i=1:1
%    plot(squeeze(x(i,1,:)),squeeze(x(i,2,:)),'LineWidth',2.0,'Color','red')
% end

N=np;
col = jet(N);
hold on
%trajectories with initial and final points highlighted. 
d = 1;
ip = 1;
ipp=1;
dd = 0;
sss= 0;
cont=1;
view1=19;
view2=48;
movim = 1;


  while dd ==0

%el if stop detiene la simulacion en 3D   
    stop=get(handles.stop,'String');
    if stop == "Stop."
       dd=1;
     continue
    end
    
    centrar = get(handles.pushbutton10,'String');
    movimiento = get(handles.pushbutton5,'String');
    
    if movim == 101
    movimiento = "no";
    set(handles.pushbutton5,"String","Reiniciar?");
    Reiniciar = get(handles.pushbutton5,'String');
    while Reiniciar == "Reiniciar?"
    Reiniciar = get(handles.pushbutton5,'String');  
    pause(0.01)
    end
    
  
        
    set(handles.slider2,'Value',1)
    movim = 1;
    
    Reinicia = get(handles.pushbutton5,'String'); 
    while Reinicia == "Play"
    Reinicia = get(handles.pushbutton5,'String');  
    pause(0.01)
    end
    
    
    end
    
    
    %%% AQUI COMIENZA CODIGO PARA GENERAR MOVIMIENTO AUTOMATICO DE LOS AGENTES
    if movimiento == "Ok!"
    set(handles.slider2,'Value',movim)
    movim = movim + 1;
    pause(0.1)
    end  
    
    view1=get(handles.slider15,'Value');
    view11=round(view1);
    view2=get(handles.slider16,'Value');
    view22=round(view2);
    s = get(handles.slider2,'Value');
    ss = round(s);
    s2=get(handles.slider7,'Value');
    ss2=round(s2);
    
    if ss==0
        ss=1;
    end
    if ss2==0
        ss2=1;
    end
pause(0.01)

%  if view ~= view2
%      cont=1;
%  end


%figure

a=get(gca,'xlim');b=get(gca,'ylim');c=get(gca,'zlim');
% set(gca,'nextplot','replacechildren');
%  if cont == 1   
%     axis([a b c])
%     view(-view1,view22) %fix perspective 
%    
% %      a=a+[-5 5];b=b+[-5 5];c=c+[-5 5];
%      grid on
%      cont = 0;
%  end
 ejex=get(handles.ejey,'Value')/10;
 ejey=get(handles.ejex,'Value')/10;
 zoom=get(handles.slider10,'Value')/10;
 cabeza_agente = get(handles.cabeza,'String');
 cabeza_agente2 = get(handles.cabezano,'String');
 salir=get(handles.salir,'value');
%  a=[-zoom+ejey zoom+ejey];b=[-zoom zoom];c=[-zoom+ejex zoom+ejex];
%  axis([a b c])
  grid on
%  if ss ~= sss
%      ss
%  end
     
%trajectories with initial and final points highlighted. 
cla
hold on

  if centrar == "Centrado"
            set(handles.ejex,'Value',0)
            set(handles.ejey,'Value',0)
            set(handles.slider10,'Value',50)/10
            set(handles.slider15,'Value',-38);
            set(handles.slider16,'Value',29);
            set(handles.pushbutton10,'String',"Centrar")
    end
%for ss=1:1:200
if cabeza_agente == "Con cabeza"
set(handles.cabeza,'visible','on'); 
set(handles.cabezano,'visible','off');
set (handles.pushbutton16, 'BackgroundColor' , 'green' ) 
for kk=1:np
    
  %comment for removing trajectories
  zoom_agente= get(handles.pushbutton6,'String');
  o = get(handles.popupmenu3,'Value');% recibe el valor del menu de agente
  front = get(handles.radiobutton11,'Value');
  back = get(handles.radiobutton12,'Value');
  
  plot3(squeeze(x(kk,1,ss2:1:ss)),squeeze(x(kk,2,ss2:1:ss)),squeeze(x(kk,3,ss2:1:ss)),'Color',col(kk,:),'LineWidth',2','LineStyle','-')
    
if zoom_agente == "Zoom Personal" & front == 1%% controla la cabeza del agente para realizar zoom de seguimiento
     set(handles.salir,'visible','on');
     set(handles.salir,'value',0);
%       a=[-zoom+ejey zoom+ejey];b=[-zoom zoom];c=[-zoom+ejex zoom+ejex];
%       axis([a b c])
%       xlim([x(o,1,ss)-zoom+ejex x(o,1,ss)+zoom+ejex])
%       ylim([x(o,2,ss)-zoom+ejey x(o,2,ss)+zoom+ejey])
     
     a=[x(o,1,ss)-zoom+ejey x(o,1,ss)+zoom+ejey];b=[x(o,1,ss)-zoom x(o,1,ss)+zoom];c=[x(o,1,ss)-zoom+ejex x(o,1,ss)+zoom+ejex];
     a1=[x(o,2,ss)-zoom+ejey x(o,2,ss)+zoom+ejey];b1=[x(o,2,ss)-zoom x(o,2,ss)+zoom];c1=[x(o,2,ss)-zoom+ejex x(o,2,ss)+zoom+ejex];
     a2=[x(o,3,ss)-zoom+ejey x(o,3,ss)+zoom+ejey];b2=[x(o,3,ss)-zoom x(o,3,ss)+zoom];c2=[x(o,3,ss)-zoom+ejex x(o,3,ss)+zoom+ejex];
     axis([a b c])
     axis([a1 b1 c1])
     axis([a2 b2 c2])
%      plot3(squeeze(x(o,1,ss)),squeeze(x(o,2,ss)),squeeze(x(o,3,ss)))
     
  elseif zoom_agente == "Zoom Personal" & back == 1
          set(handles.salir,'visible','on');
          set(handles.salir,'value',0); 
        a=[x(o,ss)-zoom+ejey zoom+ejey];b=[x(o,ss)-zoom zoom];c=[x(o,ss)-zoom+ejex zoom+ejex];
%      a1=[x(o,2,ss)-zoom+ejey zoom+ejey];b1=[x(o,2,ss)-zoom zoom];c1=[x(o,2,ss)-zoom+ejex zoom+ejex];
%      a2=[x(o,3,ss)-zoom+ejey zoom+ejey];b2=[x(o,3,ss)-zoom zoom];c2=[x(o,3,ss)-zoom+ejex zoom+ejex];
       axis([a b c])
%      axis([a1 b1 c1])
%      axis([a2 b2 c2])
       plot3(squeeze(x(o,1,ss2)),squeeze(x(o,2,ss2)),squeeze(x(o,3,ss2))) 
            
            
  else
 
 if salir == 1
  a=[-zoom+ejey zoom+ejey];b=[-zoom zoom];c=[-zoom+ejex zoom+ejex];
  axis([a b c])
 end   
  end
%   plot3(squeeze(x(kk,1,1)),squeeze(x(kk,2,1)),squeeze(x(kk,3,1)),'Color',[0.5 0.5 1],'Marker','x','MarkerSize',5)
%   plot3(squeeze(x(kk,1,ss)),squeeze(x(kk,2,ss)),squeeze(x(kk,3,ss)),'Color',[sqrt(1-1)/N sqrt(1-1)/N sqrt(1-1)/N],'Marker','square','MarkerSize',10,'MarkerFaceColor',col(kk,:))
% plot3(squeeze(x(kk,1,ss)),squeeze(x(kk,2,ss)),squeeze(x(kk,3,ss)),'Color',col(kk,:))
end
end
if cabeza_agente == "ok"
set(handles.cabeza,'visible','off'); 
set(handles.cabezano,'visible','on');
%Menu2 es para mostrar en una lista las posiciones de los agentes
menu2 = [1:np];
    set(handles.popupmenu6,'string',menu2) 
%Menu3 almacenara las posiciones espaciales de cada agente    
menu3=[];

 for  kk=1:np 
  a=[-zoom+ejey zoom+ejey];b=[-zoom zoom];c=[-zoom+ejex zoom+ejex];
  axis([a b c])
  plot3(squeeze(x(kk,1,ss)),squeeze(x(kk,2,ss)),squeeze(x(kk,3,ss)),'Marker','o','MarkerSize',12,'MarkerFaceColor',col(kk,:))
  menu3(kk,:)=[x(kk,1,ss) x(kk,2,ss) x(kk,3,ss)];
 end
v=get(handles.popupmenu6,'Value');
set(handles.text22,'string',ceil(menu3(v,1))+" "+ceil(menu3(v,2))+" "+ceil(menu3(v,3)))

end
pause(0.01)
 
 
 
  view(-view11,view22)
 %   view(-5,10)
% 
% 
axis([a b c])
sss=ss;
% d=0;
view2=view1;    
%end
menu = [1:np]; 
    set(handles.popupmenu3,'string',menu)
end    

% end
cla reset
end
%******** FIN CODIGO 3D *****************




%%%%%%%%%%%%%%%%Otras variables%%%%%%%%%%%%%%%%%%%%%%%%%%5
function text2_CreateFcn(hObject, eventdata, handles)
%
function slider2_Callback(hObject, eventdata, handles)
%
function slider2_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
%
function play_Callback(hObject, eventdata, handles)
%
function pausa_Callback(hObject, eventdata, handles)
p=get(handles.pausa,'Value');
%
function slider3_Callback(hObject, eventdata, handles)
%
function slider3_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
%
function edit1_Callback(hObject, eventdata, handles)
%
function edit1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%
function edit2_Callback(hObject, eventdata, handles)
%
function edit2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%
function text6_CreateFcn(hObject, eventdata, handles)
%
function stop_Callback(hObject, eventdata, handles)
global cuadrado bandada poligon
set(handles.original,'Value',1);
set(handles.uitable10,'Visible','off')
set (handles.pushbutton16, 'BackgroundColor' , 'red' ) 
set(handles.salir,'visible','off');
set(handles.slider2,'Value',0)
set(handles.slider7,'Value',0)
set(handles.radiobutton8,'Value',1)
set(handles.text8,'String','Ingrese nuevos valores')
set(handles.pushbutton6,'Value',0)
set(handles.pushbutton6,'String',"Stop")
set(handles.pushbutton5,'Value',0)
set(handles.pushbutton5,'String',"Stop")
set(handles.popupmenu3,'visible','off')
set(handles.text13,'visible','off')
set(handles.uibuttongroup3,'visible','off')
set(handles.pushbutton9,'visible','off')
set(handles.slider15,'Visible','off');
set(handles.slider16,'Visible','off');
set(handles.text21,'Visible','off');
% cuadrado = 0;
% poligon = 0;
% bandada = 0;

set(handles.stop,'String','Stop.')
set(handles.cabeza,'String','Cabeza');
set(handles.cabeza,'visible','off'); 
set(handles.cabezano,'visible','off');

% a=get(gca,'xlim');b=get(gca,'ylim')
% axis([a b])
% a=a+[-2 2];b=b+[-2 2];  
% for i=0;3
% cla reset
% cla reset
% pause(0.01)
% end

cla reset



%
function radiobutton8_Callback(hObject, eventdata, handles)
%
function text8_CreateFcn(hObject, eventdata, handles)
%
function slider7_Callback(hObject, eventdata, handles)
%
function slider7_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
%
function text9_CreateFcn(hObject, eventdata, handles)
%
function nosotros_Callback(hObject, eventdata, handles)
%
function archivo_Callback(hObject, eventdata, handles)
%
function grabar_Callback(hObject, eventdata, handles)
%
function Untitled_1_Callback(hObject, eventdata, handles)
%
function Untitled_2_Callback(hObject, eventdata, handles)
%
function natural_Callback(hObject, eventdata, handles)
%
function personalizado_Callback(hObject, eventdata, handles)
set(handles.uitable10,'Visible','on');

function descargar_Callback(hObject, eventdata, handles)
%
function slider8_Callback(hObject, eventdata, handles)
%
function slider8_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
%
function menu_Callback(hObject, eventdata, handles)
%
function menu_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%
function pushbutton4_Callback(hObject, eventdata, handles)
%
function popupmenu3_Callback(hObject, eventdata, handles)
m=get(handles.popupmenu3,'Value');
%
function popupmenu3_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%
function slider9_Callback(hObject, eventdata, handles)
%
function slider9_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
%
function slider10_Callback(hObject, eventdata, handles)
%
function slider10_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
%
function pushbutton5_Callback(hObject, eventdata, handles)
Reiniciarr = get(handles.pushbutton5,'String');
if Reiniciarr == "Reiniciar?"
set(handles.pushbutton5,'String',"Play?")
elseif Reiniciarr == "Play?"
set(handles.pushbutton5,'String',"Ok!") 
else
set(handles.pushbutton5,'String',"Ok!") 
end


%
function radiobutton10_Callback(hObject, eventdata, handles)
%
function pushbutton6_Callback(hObject, eventdata, handles)
set(handles.pushbutton6,'String',"Zoom Personal")
set(handles.popupmenu3,'visible','on')
set(handles.text13,'visible','on')
set(handles.uibuttongroup3,'visible','on')
set(handles.radiobutton11,'Value',1)
%
function ejey_Callback(hObject, eventdata, handles)
%
function ejey_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
%
function ejex_Callback(hObject, eventdata, handles)
%
function ejex_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
%
function radiobutton11_Callback(hObject, eventdata, handles)

% --- Executes during object deletion, before destroying properties.
function uitable7_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to uitable7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
function axes1_DeleteFcn(hObject, eventdata, handles)


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global np ndim sigma K beta pp flag alpha M zdes tf
f = uifigure('Position', [100 100 300 265],'Color','White','Name','Obtención de datos');
    t = uitable(f,'Data',[np;ndim;alpha;beta;flag;M;sigma;K;pp;tf]);
    t.FontSize = 10;%alpha beta flag M sigma K pp   ;'Alpha' ;'Beta' ; 'Flag';'M' ;'Sigma' ;'K' ;'PP'
    t.ColumnName = {'Datos'};
    t.RowName = {'Agentes' 'Dimensión' 'Alpha' 'Beta' 'Flag' 'M' 'Sigma' 'K' 'PP' 'Tiempo final'};
    t.Position = [50 50 195 205];


% --- Executes when figure1 is resized.
function figure1_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
set(handles.pushbutton10,'String',"Centrado")


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.pushbutton11,'String',"Ok!")



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double


% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in original.
function original_Callback(hObject, eventdata, handles)
set(handles.uitable10,'Visible','off');


% --- Executes on button press in deseado.
function deseado_Callback(hObject, eventdata, handles)
global np ndim sigma K beta pp flag alpha M zdes dt
dos=get(handles.radiobutton26,'Value');
tres=get(handles.radiobutton27,'Value');


if dos==1
    set(handles.uitable10,'Visible','on');
    A = {'Alpha';'Beta';'Sigma';'K';'PP';'Flag';'M';'Tiempo Final';'Dif. tiempo'; ' '; 'Influence';'Influenceu'};
    B = {'1.1';'0.4';'0';'100';'2';'0';'50';'20';'.001';' ';'1./((sigma+distance).^pow)';'1./((1+distance.^2).^pow'};
    variables = [A B];
    set(handles.uitable10,'data',variables);
end

if tres == 1
    set(handles.uitable10,'Visible','on');
    A = {'Alpha';'Beta';'Sigma';'K';'PP';'Flag';'M';'Tiempo Final';'Dif. tiempo';' '; 'Influence';'Influenceu'};
    B = {'1.1';'0.1';'0';'10';'4';'0';'50';'400';'0.05';' ';'1./((sigma+distance).^pow)';'1./((1+distance.^2).^pow'};
    variables = [A B];
    set(handles.uitable10,'data',variables);
end



% --- Executes when entered data in editable cell(s) in uitable10.
function uitable10_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to uitable10 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu4.
function popupmenu4_Callback(hObject, eventdata, handles)
%%Menu desplegable para figuras de agentes %%%%
global cuadrado bandada poligon elegir circulo triangulo
elegir = 0;
cuadrado = 0;
bandada = 0;
poligon = 0;
circulo = 0;
triangulo = 0;
v=get(handles.popupmenu4,'Value');
%bandada = get(handles.popupmenu4,'Value');

switch v
    
        
    case 2
        cuadrado = 1;
    case 3
        bandada = 1;
    case 4
        poligon = 1;
    case 5
        circulo = 1;
    case 6
        triangulo = 1;
    otherwise
        
    elegir = 1;
        
end

% --- Executes during object creation, after setting all properties.
function popupmenu4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit10_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)



% --- Executes on slider movement.
function slider15_Callback(hObject, eventdata, handles)
% hObject    handle to slider15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider15_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider16_Callback(hObject, eventdata, handles)
% hObject    handle to slider16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end




% --- Executes during object deletion, before destroying properties.
function pushbutton13_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in cabeza.
function cabeza_Callback(hObject, eventdata, handles)
set(handles.cabeza,'String','ok');



% --- Executes on button press in cabezano.
function cabezano_Callback(hObject, eventdata, handles)
set(handles.cabeza,'String','Con cabeza');


% --- Executes on button press in ecuaciones.
function ecuaciones_Callback(hObject, eventdata, handles)
% hObject    handle to ecuaciones (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of ecuaciones


% --- Executes on button press in salir.
function salir_Callback(hObject, eventdata, handles)
% hObject    handle to salir (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of salir




% --- Executes on selection change in popupmenu6.
function popupmenu6_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu6 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu6


% --- Executes during object creation, after setting all properties.
function popupmenu6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu7.
function popupmenu7_Callback(hObject, eventdata, handles)
global cuadrado paredes circulo elegir bandada poligon olympic piramide
elegir = 0;
cuadrado = 0;
paredes = 0;
circulo = 0;
bandada = 0;
olympic = 0;
piramide = 0;
v=get(handles.popupmenu7,'Value');
%bandada = get(handles.popupmenu4,'Value');

switch v
    
        
    case 2
        cuadrado = 1;
    case 3
        paredes = 1;
    case 4
        circulo = 1;
    case 5
        piramide=1;
    case 6
        bandada=1;
    case 7   
        poligon=1;
    case 8
        olympic=1;
        
    otherwise
        
    elegir = 1;
        
end

% --- Executes during object creation, after setting all properties.
function popupmenu7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu8.
function popupmenu8_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu8 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu8


% --- Executes during object creation, after setting all properties.
function popupmenu8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in radiobutton26.
function radiobutton26_Callback(hObject, eventdata, handles)
set(handles.popupmenu4,'Visible','on')
set(handles.popupmenu7,'Visible','off')
set(handles.original,'Value',1);
original=get(handles.original,'Value');

if original == 1
   set(handles.uitable10,'Visible','off') 
end


% --- Executes on button press in radiobutton27.
function radiobutton27_Callback(hObject, eventdata, handles)
set(handles.popupmenu4,'Visible','off')
set(handles.popupmenu7,'Visible','on')
set(handles.original,'Value',1); 
original=get(handles.original,'Value');

if original == 1
   set(handles.uitable10,'Visible','off') 
end

% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%******************************************************************
%******************INCORPORACION DE CODIGOS EXTERIORES*************

%**********************CODIGOS PARA 2D************************************

function [x]=csdynmatlab(t,xinitt)
global np ndim beta pp flag alpha zdes M K voldvi xnodevi aoldxvi aoldyvi An


xinit=reshape(xinitt,2*np,ndim);
a=zeros(np,np,ndim);
a2=zeros(np,np,ndim);
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
    a(:,:,k)=K*influence(distance,alpha).*difv(:,:,k); %full observation influence
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
x(1:np,:)=v0;
x(np+1:2*np,:)=squeeze(sum(a,2))./Np+0*alignment*squeeze(sum(f,2))+1*u;
x=x(:);
%  end

%*****************FIN CODIGO csdynmatlab PARA 2D*********************

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
% end

%*****************FIN CODIGO csdynshape PARA 2D**************





%****************CODIGOS PARA 3D*******************************************
function [x]=csdynshape3d(xinit,u)
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
%end

%***************************FIN csdynshape3d***********************

function [x]=RK4(x0,u,dt,fun)
global np ndim
x=zeros(2*np,ndim);

   p1=feval(fun,x0,u);
   p2=feval(fun,x0+(dt/2)*p1,u);
   p3=feval(fun,x0+(dt/2)*p2,u);
   p4=feval(fun,x0+(dt)*p3,u);
   x=x0+dt*((1/6)*p1+(1/3)*p2+(1/3)*p3+(1/6)*p4);
   %x(:,:,k+1)=x(:,:,k)+dt*p1;

   
 %*************FIN RK43d ********************************************
 
 function [a]=influenceu(distance,pow)
% recibido=get(handles.uitable10,'data');
%valores de la tabla
%     v2 = recibido(12,2);
%     influenceuu = str2double(v2);
    a=1./((1+distance.^2).^pow);
%     a=influenceuu;
% 
%  end

%**************** FIN INFLUENCEU3D*********************************
function [a]=influence(distance,pow)
global sigma 
% recibido=get(handles.uitable10,'data');
if distance==0
    a=0;
else
    %valores de la tabla
%     v1 = recibido(11,2);
%     influencee = str2double(v1);
    a=1./((sigma+distance).^pow);
%     a = influencee;
end

%*****************FIN INFLUENCE3D************************

function [f]=repulsion(distance,pow)

f=1./(distance-1e-10).^pow;
% end

%****************FIN REPULSION***************************


% --- Executes on button press in radiobutton28.
function radiobutton28_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% dos=get(handles.radiobutton26,'Value');
% tres=get(handles.radiobutton27,'Value');
% ecuaciones=get(hanles.radiobutton28,'Value')
% 
% if ecuaciones == 0
% 
% if dos==1
%     set(handles.uitable10,'Visible','on');
%     A = {'influence';'Beta';'Sigma';'K';'PP';'Flag';'M';'Tiempo Final';'Dif. tiempo'};
%     B = {'1.1';'0.4';'0';'100';'2';'0';'50';'20';'.001'};
%     variables = [A B];
%     set(handles.uitable10,'data',variables);
% end
% 
% if tres == 1
%     set(handles.uitable10,'Visible','on');
%     A = {'Alpha';'Beta';'Sigma';'K';'PP';'Flag';'M';'Tiempo Final';'Dif. tiempo'};
%     B = {'1.1';'0.1';'0';'10';'4';'0';'50';'400';'0.05'};
%     variables = [A B];
%     set(handles.uitable10,'data',variables);
% end
% end
