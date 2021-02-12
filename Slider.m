


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
%24
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Slider

% Last Modified by GUIDE v2.5 10-Feb-2021 00:12:01

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
 global vectoresAgentes vector grabar velocidadAgentes vector2;
 vectoresAgentes = [];
 grabar = 0;

 vector = 1;
 
 velocidadAgentes = [];
 

 vector2 = 1;
 
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
 global paredes str2 distance pow circulo velocidadAgentes str3 str4 triangulo piramide olympic dt str1 original deseado creativo cargar vectoresAgentes vector vector2 refreshOn grabar cargarzdes
 %%% ocultar menu de zoom personal al apretar run sin apretar stop
 set(handles.salirzoom,'String','Salir zoom')
set(handles.pushbutton6,'String','Zoom personal')
set(handles.popupmenu3,'visible','off')
set(handles.text13,'visible','off')
set(handles.uibuttongroup3,'visible','off')
set(handles.radiobutton11,'Value',0)
set(handles.pushbutton6,'visible','on')
set(handles.salirzoom,'visible','off')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
 
 
 % reestablece nombre boton stop
% clear all
% clc
iniciales = get(handles.iniciales,'Value');
refreshOn = 1;
set(handles.popupmenu6,'Visible','on')
set(handles.text22,'Visible','on')
set(handles.salirzoom,'String','Salir zoom')

vectoresAgentes;
tamanio = size(vectoresAgentes);
fila = tamanio(1,1);
columna =tamanio(1,2);
restr = tamanio(1,1);
conta = [];
conta2 = [];
conta3 = [];
contador = 0;
for i=1:fila
    conta = vectoresAgentes(i,:);
    for j=1:columna
        conta2 = conta(1,j);
%         conta3 = conta(j,1)
        if isnan(conta2)
            contador = 1;
        end
    end
end

if iniciales == 1
    velocidadAgentes;
    tamanio2 = size(velocidadAgentes);
    fila2 = tamanio2(1,1);
    columna2 =tamanio2(1,2);
    restr2 = tamanio2(1,1);
    contav = [];
    contav2 = [];
    contav3 = [];
    contador2 = 0;
    for i=1:fila
        contav = velocidadAgentes(i,:);
        for j=1:columna2
            contav2 = contav(1,j);
    %         conta3 = conta(j,1)
            if isnan(contav2)
                contador2 = 1;
            end
        end
    end
end
% for i=1:columna;
%     conta2 = vectoresAgentes(:,i)
% end

% str1=get(handles.edit11,'String');
 
set(handles.pushbutton16, 'BackgroundColor' , 'yellow' );
set(handles.stop,'String',"Stop")

npp    = get(handles.edit1,'String');%Se guarda el vaor que se ingrese en el cuadro de texto en guide en la variable "npp"
nppp   = str2double(npp); %el valor de npp se convierte en un numer

if nppp <= 1
    set(handles.text8,'String','Ingrese valor mayor a 1')
    set (handles.pushbutton16, 'BackgroundColor' , 'fred' ) 
    set(handles.stop,'String','Stop.')
end
if isnan (nppp)
    set(handles.text8,'String','Ingrese valor numerico')
    set (handles.pushbutton16, 'BackgroundColor' , 'red' )
    set(handles.stop,'String','Stop.')
end

if (tamanio(1,1)~= (nppp-1)|| contador ==1 ) && (get(handles.popupmenu7,'Value')== 9 || get(handles.popupmenu4,'Value') == 7)
    vectoresAgentes = [];
    vector = 1;
    tamanio = [];
    restr = 0;
    set(handles.text8,'String','Ingrese la cantidad correcta de vectores')
    set (handles.pushbutton16, 'BackgroundColor' , 'red' )
    set(handles.stop,'String','Stop.')
    
else

    set(handles.text8,'String',' ')



    %%Codigo para ingresar cantidad de agentes 
dos = get(handles.radiobutton26,'Value');
tres= get(handles.radiobutton27,'Value');



if dos ==1;
       


a=get(gca,'xlim');b=get(gca,'ylim');c=get(gca,'zlim');
axis([a b])
a=a+[-2 2];b=b+[-2 2];     
    
flagcolision=0;
tic
original= get(handles.original,'Value');
deseado= get(handles.deseado,'Value');


if original == 1
str1=@(distance,pow)1./((1+distance.^2).^pow); 

% if distance == 0
%     
%     str2 = @(distance,pow)0;
% else
str2 =@(distance,pow)1./((0+distance).^pow);

% end

slider = 20000;
slider2 = 100;
slider3 = 100;
slider4 = 100;
solver = "ode23s";
  set(handles.slider2,'MIN',0,'MAX',slider);
  set(handles.slider7,'MIN',0,'MAX',slider);
  set(handles.slider10,'MIN',1,'MAX',slider4);
  set(handles.ejey,'MIN',-slider2,'MAX',slider2);
  set(handles.ejex,'MIN',-slider3,'MAX',slider3);
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
    
  
flag=0;
  recibido=get(handles.uitable10,'data');
  
infl = recibido(12,2);
str1 = @(distance,pow)eval(infl{1});


if distance == 0
    
    str4 = @(distance,pow)0;
else
str2 = recibido(11,2);
% str3 = eval(str2{1});
str4 = @(distance,pow)eval(str2{1});

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
  dt = str2double(v8);
  v9 = recibido(13,2);
  slider = str2double(v9);
  v10 = recibido(14,2);
  slider2 = str2double(v10);
  v11 = recibido(15,2);
  slider3 = str2double(v11);
  v12 = recibido(16,2);
  slider4 = str2double(v12);
  solver = recibido(10,2);
  
  slider = 20000;
    slider2 = 100;
    slider3 = 100;
    slider4 = 100;
  set(handles.slider2,'MIN',0,'MAX',slider);
  set(handles.slider7,'MIN',0,'MAX',slider);
  set(handles.slider10,'MIN',1,'MAX',slider4);
  set(handles.ejey,'MIN',-slider2,'MAX',slider2);
  set(handles.ejex,'MIN',-slider3,'MAX',slider3);
end    
    

    
npp    = get(handles.edit1,'String');%Se guarda el vaor que se ingrese en el cuadro de texto en guide en la variable "npp"
nppp   = str2double(npp); %el valor de npp se convierte en un numer
if nppp <= 1
    set(handles.text8,'String','Ingrese valor mayor a 1')
    set (handles.pushbutton16, 'BackgroundColor' , 'red' )   
    set(handles.stop,'String','Stop.')
elseif isnan (nppp)
    set(handles.text8,'String','Ingrese valor numerico')
    set (handles.pushbutton16, 'BackgroundColor' , 'red' )   


else    
    
% set(handles.text8,'String',' ')
%%%%%%%
% close all
% clc
%function [value,th]=consensus3(gamma0,lambda0)
set(handles.text8,'String','')
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
iniciales = get(handles.iniciales,'Value');

if iniciales == 0
    for i=1:np
        for kk=1:ndim
            x00(i,kk)=10*rand-1;
            v00(i,kk)=2*rand-1;

        end
    end
end

if iniciales == 1
    
  x00=[vectoresAgentes]; 
  v00=[velocidadAgentes];
% x00=[vectoresAgentes];
% for i=1:np
%         for kk=1:ndim
% 
%             v00(i,kk)=2*rand-1;
% 
%         end
%     end
end



vbar0=1/np*sum(v00,1);
vbar2=1/np*sum(v00,1);
zdes=zeros(np-1,ndim);

%%%% en caso de elegir la primera opcion de popmenu %%%
set (handles.pushbutton16, 'BackgroundColor' , 'yellow' )  
if elegir == 1;
    set(handles.text8,'String','Elegir forma de agente')
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if triangulo == 1
    
%     val1=0;
%     val2=5;
%     val3 = floor(np/2);
%     val4= vbar2(2), vbar2(1);
%     zdes=[-0.5 -0.5;-0.5 -0.5;0 -0.5;0 -0.5;2 0;0 0.5;0 0.5;-0.5 0.5;-2.5 -1.5;-0.5 0;-0.5 0;-0.5 0;-0.5 0;1 0.5;0 0.5;0 0.5;0 0.5;-2 0;-1 0;-1 0;2 -0.5;0 -0.5;-1 0;-1 0;2 -0.5;0 -0.5;-1 0;-1 0;-2 0;0 0.5;0 0.5;0 0.5;0 0.5;-1 -1.5;-1 0.5;-1 -0.5;-1 -0.5;0 0.5;0 0.5;0 0.5;0 0.5;-1 -2];
 %   zdes=[0 0.5;-1 0;1 0.5;0 0.5;0 0.5;-2 -1;-1 -0.5;0 0.5;0 0.5;0 0.5;0 0.5;0.5 -1.75];
   %zdes=[0 2.5;0 5;0 2.5;-1 1.25;-1 1.25; -1 -1.25;-1 -1.25;0 -2.5;0 -2.5;0 -2.5]; %letra U
%     zdes= [-1.25 0;-1.25 0;-1.2 0;-1.25 -2.5;2.5 5;0 2.5;0 2.5;0 1.25;0 1.25];%Letra T
%       zdes=[-1.5 0;-1.5 0;3 2.5;0 2.5;-1.5 0;-1.5 0;3 2.5;0 2.5;-1.5 0;-1.5 0];%letra E
%     zdes=[0 -2.5;0 -2.5;0 -5;-1.5 2.5;-1.5 2.5;-1.5 0;-2.5 -5;0 5;0 2.5;0 2.5];%letra M
%     zdes=[0 2.5;0 5;0 2.5;-1 1.25;-1 1.25; -1 -1.25;-1 -1.25;0 -2.5;0 -2.5;0 -2.5;-0.5 0;-1.25 0;-1.25 0;-1.2 0;-1.25 -2.5;2.5 5;0 2.5;0 2.5;0 1.25;0 1.25];% letra UT
%     zdes=[0 2.5;0 5;0 2.5;-1 1.25;-1 1.25; -1 -1.25;-1 -1.25;0 -2.5;0 -2.5;0 -2.5;-0.5 0;-1.25 0;-1.25 0;-1.2 0;-1.25 -2.5;2.5 5;0 2.5;0 2.5;0 1.25;0 1.25;-5 -10;-1.5 0;-1.5 0;3 2.5;0 2.5;-1.5 0;-1.5 0;3 2.5;0 2.5;-1.5 0;-1.5 0];% letra UTE
%     zdes=[0 2.5;0 5;0 2.5;-1 1.25;-1 1.25; -1 -1.25;-1 -1.25;0 -2.5;0 -2.5;0 -2.5;-0.5 0;-1.25 0;-1.25 0;-1.2 0;-1.25 -2.5;2.5 5;0 2.5;0 2.5;0 1.25;0 1.25;-5 -10;-1.5 0;-1.5 0;3 2.5;0 2.5;-1.5 0;-1.5 0;3 2.5;0 2.5;-1.5 0;-1.5 0;-2 0;0 -2.5;0 -2.5;0 -5;-1.5 2.5;-1.5 2.5;-1.5 0;-2.5 -5;0 5;0 2.5;0 2.5;-23 10];% letra UTEM
% zdes = [-6 0;6 -2.5;-6 0;6 -2.5;-6 0;6 -2.5;-6 0;6 -2.5;-6 0;4.5 2.5;-3 0;1.5 2.5] %M

%palabra utem completa
% zdes=[1 -1.25;-2 0;3 -1.25;-4 0;4 -2.5;-4 0;4 -2.5;-4 0;4 -2.5;-4 0;-5 10;0 -2.5;0 -2.5;0 -2.5;0 -2.5;1.25 0;-2.5 0;3.75 0;-5 0;-2.5 0;-1.5 0;-1.5 0;3 2.5;0 2.5;-1.5 0;-1.5 0;3 2.5;0 2.5;-1.5 0;-1.5 0;-2 0;-6 0;6 -2.5;-6 0;6 -2.5;-6 0;6 -2.5;-6 0;6 -2.5;-6 0;4.5 2.5;-3 0;1.5 2.5];% letra UTEM
filename = 'zdes.txt';
[zdes,delimiterOut]=importdata(filename);
zdes
% delimiterIn = ' ';
% headerlinesIn = 1;
% zdes = importdata(filename,delimiterIn,headerlinesIn)


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
        zdes(conta,:) = [ -3  uno];
%         zdes(conta,:) = [ -3 uno];
        conta=conta+1;
        contador = 0;
        zdess = 0;
        dif = dif +-3;
    end 
end
end
%%
if creativo == 1

    zdes = [vectoresAgentes];
end

if cargarzdes == 1
filename = 'zdes.txt';
[zdes,delimiterOut]=importdata(filename);
zdes
end
%%
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
set (handles.pushbutton16, 'BackgroundColor' , 'yellow' )  
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
zdes
end

if cargar == 1;
% x1 = load ('x1.mat')
% x2 = load ('x2.mat')
% x = load('x.mat') 
% x1 = load ('x1.txt')
% x2 = load ('x2.txt')
% x = load('x.txt') 
cargarArchivotxt = get(handles.edit18,'String');
archivo1 = strcat(cargarArchivotxt,'1.txt')
archivo2 = strcat(cargarArchivotxt,'2.txt')
archivo3 = strcat(cargarArchivotxt,'.txt')
% filename = 'x1.txt';
delimiterIn = ' ';
headerlinesIn = 1;
x1 = importdata(archivo1,delimiterIn,headerlinesIn);
% filename = 'x2.txt';
delimiterIn = ' ';
headerlinesIn = 1;
x2 = importdata(archivo2,delimiterIn,headerlinesIn);
% filename = 'x.txt';
delimiterIn = ' ';
headerlinesIn = 1;
x = importdata(archivo3,delimiterIn,headerlinesIn);
% delimiterIn = ' ';
% headerlinesIn = 1;
% solver = importdata('solver.txt',delimiterIn,headerlinesIn);

else
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initial energies and gamma lambda
iniciales = get(handles.iniciales,'Value');
if iniciales == 0 
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
    % scaling to choose gamma lambda at t=0
    x00=sqrt(gamma0/Gamma)*x00;
    v00=sqrt(lambda0/Lambda)*v00;
    Gammap=0;
    Lambdap=0;
    Ec0p=0;
    D0=0;
    set (handles.pushbutton16, 'BackgroundColor' , 'yellow' )
    for i=1:np
        Ec0p=Ec0p+norm(v00(i,:),2).^2; %kinetic
        for j=1:np
            Gammap=Gammap+norm(x00(i,:)-x00(j,:),2).^2;
            Lambdap=Lambdap+norm(v00(i,:)-v00(j,:),2).^2;
        if original == 1
            D0=D0+0.5*(K/np)*str2(norm(x00(i,:)-x00(j,:),2),alpha)*norm(v00(i,:)-v00(j,:),2)^2;
        end

        if deseado == 1
           D0=D0+0.5*(K/np)*str4(norm(x00(i,:)-x00(j,:),2),alpha)*norm(v00(i,:)-v00(j,:),2)^2;
        end
        end
    end
    Ep0=0;
    for i=2:np
        t=norm(x00(i-1,:)-x00(i,:)-zdes(i-1,:),2).^2;
        Ep0=Ep0+M*(sqrt(t+1)-sqrt(1)); %potential for beta=0.5
    end

    Gammap=1/(2*np^2)*Gammap;
    Lambdap=1/(2*np^2)*Lambdap;
end

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
set (handles.pushbutton16, 'BackgroundColor' , 'yellow' )

tic
       
if solver == "ode23s"
        %ode23s%%%
        xinitt=[x00(:,1);v00(:,1);x00(:,2);v00(:,2)];
        [t,X]=ode23s(@csdynmatlab,0:dt:tf,xinitt); % resuelve el sistema de ecuaciones diferenciales con el solver "ode23"
        X=X';
        ftime=length(t);
        x1=X(1:np,:); 
        x2=X(2*np+1:3*np,:);
        v1=X(np+1:2*np,:); % desde np + 1 hasta 2np tenemos la velocidad en el ejex
        v2=X(3*np+1:4*np,:);
        %%%%%%%
        
        [xx2,yy2,zz2] = size(x);
        xx2;
        yy2;
        zz2;
        if zz2 <= slider

        set(handles.slider2,'MIN',0,'MAX',zz2);
        set(handles.slider7,'MIN',0,'MAX',zz2);

        end
end


if solver == "RK4" || solver == "rk4"
            x1=[];
            x2=[];
            %RK
            for i=1:nsteps-1
     

                x(:,:,i+1)=RK4(x(:,:,i),u(:,:,i),dt,@csdynshape);
                x0=squeeze(x(1:np,1,i+1));
                x1=[x1 x0];
                x0=squeeze(x(1:np,2,i+1));
                x2=[x2 x0];
                
            
            end

            [xx2,yy2,zz2] = size(x1);
          
            xx2;
            yy2;
            zz2;
            if yy2 <= slider 

            set(handles.slider2,'MIN',0,'MAX',yy2);
            set(handles.slider7,'MIN',0,'MAX',yy2);

            end
end
end        
toc

if grabar == 1
palabra = get(handles.edit17,'String')
palabra1 = strcat(palabra,'1.txt')
palabra2 = strcat(palabra,'2.txt')
palabra3 = strcat(palabra,'.txt')

save(palabra1,'x1')
save(palabra2,'x2')
save(palabra3,'x')
save('solver.txt','solver')
% save('x2.txt','x2')
% save('x.txt','x')
end
grabar = 0;
% trabajo_titulo
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
 set (handles.pushbutton16, 'BackgroundColor' , 'yellow' )     

 

 
%while2D
 while   dd==0
     global columna2d
 [fila,columna2d]=size(x1);

       %Menu2 es para mostrar en una lista las posiciones de los agentes
menu2 = [1:np];
    set(handles.popupmenu6,'string',menu2) 
%Menu3 almacenara las posiciones espaciales de cada agente    
menu3=[]; 
   
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
    
    set(handles.slider2,'Value',1);
    movim = 1;
    
    Reinicia = get(handles.pushbutton5,'String'); 
    while Reinicia == "Play"
    Reinicia = get(handles.pushbutton5,'String');  
    pause(0.01)
    end
    
end

%%% AQUI COMIENZA CODIGO PARA GENERAR MOVIMIENTO AUTOMATICO DE LOS AGENTES
if movimiento == "Ok!"
    set(handles.slider2,'Value',movim);
    movim = movim + 200;
    pause(0.1)
end    

        s= get(handles.slider2,'Value');
        s2=get(handles.slider7,'Value');
        ss=round(s);%ss es la variable que se genera con el slider y se redondea
        ss2=round(s2);
        
        if ss >= columna2d
            ss = columna2d;
            set(handles.slider2,'Value',ss);
            set(handles.slider2,'MIN',0,'MAX',ss);
        end
        
        
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
 menu3=[];  
 estela = get(handles.estela,'Value');
for     i=1:np
        
          ejey = get(handles.ejey,'Value');
          ejex = get(handles.ejex,'Value');

          
          
        set (handles.pushbutton16, 'BackgroundColor' , 'green' )
        if estela == 0
            plot(x1(i,ss2:1:ss),x2(i,ss2:1:ss),'LineWidth',2,'LineStyle','-')
            plot(x1(i,ss),x2(i,ss),'Marker','o','MarkerSize',10,'MarkerFaceColor',col(i,:))
            plot(x1(i,ss2),x2(i,ss2),'Marker','.','MarkerSize',14,'MarkerFaceColor',col(i,:))
        end
        
        if estela == 1
            if ss<800
                plot(x1(i,ss2:1:ss),x2(i,ss2:1:ss),'LineWidth',2,'LineStyle','-')
            end
            if ss > 800
                plot(x1(i,ss-800:200:ss),x2(i,ss-800:200:ss),'LineWidth',2,'LineStyle','-')
            end
            
            plot(x1(i,ss),x2(i,ss),'Marker','o','MarkerSize',10,'MarkerFaceColor',col(i,:))
        end 
        
        
        %Para registras
        %Se generan Menu2 y Menu 3 para sobreescribir la tabla
         %Menu2 es para mostrar en una lista las posiciones de los agentes
          menu2 = [1:np];
%          set(handles.popupmenu6,'string',menu2) 
%         Menu3 almacenara las posiciones espaciales de cada agente    
          for j=1:np         
          menu3(j,:)=[x1(j,ss) x2(j,ss)];
          end   
           v=get(handles.popupmenu6,'Value');
       
           set(handles.text22,'string',"x = "+(menu3(v,1)+";  "+"y = "+(menu3(v,2))));
       
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
set(handles.popupmenu6,'Visible','off')
set(handles.text22,'Visible','off')
end
cla reset
end


%******** COMIENZO CODIGO 3D*************
if tres == 1
flagcolision=0;
tic
slider = 101;
slider2 = 100;
slider3 = 100;
slider4 = 100;
original= get(handles.original,'Value');
deseado= get(handles.deseado,'Value');

vectoresAgentes;
tamanio = size(vectoresAgentes);
tamanio(1,1);

if original == 1

str1=@(distance,pow)1./((1+distance.^2).^pow); 

if distance == 0
    
    str2 = @(distance,pow)0;
else
% str2 =@(distance,pow)1./((sigma+distance).^pow);
str2 =@(distance,pow)1./((0+distance).^pow);
end

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
solver = "RK4";
end
%OPCION DE PERSONALIZADO 
if deseado == 1
  recibido=get(handles.uitable10,'data');
  infl = recibido(12,2);
  
str1 = @(distance,pow)eval(infl{1});


if distance == 0
    
    str4 = @(distance,pow)0;
else
str2 = recibido(11,2);
% str3 = eval(str2{1});
str4 = @(distance,pow)eval(str2{1});

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
  dt = str2double(v8);
  v9 = recibido(13,2);
  slider = str2double(v9);
  v10 = recibido(14,2);
  slider2 = str2double(v10);
  v11 = recibido(15,2);
  slider3 = str2double(v11);
  v12 = recibido(16,2);
  slider4 = str2double(v12);
  solver =recibido(10,2);
  
end

% set(handles.salir,'visible','off');
% set(handles.salir,'value',1);
set(handles.cabeza,'visible','on');
set(handles.cabeza,'String','Con cabeza');
set(handles.cabezano,'String','Sin cabeza');
set(handles.slider2,'MIN',0,'MAX',slider);
set(handles.slider7,'MIN',0,'MAX',slider);
set(handles.slider10,'MIN',1,'MAX',slider4);
set(handles.ejey,'MIN',-slider2,'MAX',slider2);
set(handles.ejex,'MIN',-slider3,'MAX',slider3); 
set(handles.slider15,'Value',-38);
set(handles.slider16,'Value',29);



npp    = get(handles.edit1,'String');%Se guarda el vaor que se ingrese en el cuadro de texto en guide en la variable "npp"
nppp   = str2double(npp); %el valor de npp se convierte en un numer

if nppp <= 1
    set(handles.text8,'String','Ingrese valor mayor a 1')
    set (handles.pushbutton16, 'BackgroundColor' , 'red' )   
elseif isnan (nppp)
    set(handles.text8,'String','Ingrese valor numerico')
    set (handles.pushbutton16, 'BackgroundColor' , 'red' )   


else
%%%%%%%
% close all
% clc
%function [value,th]=consensus3(gamma0,lambda0)
set(handles.text8,'String','')
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
 

np=nppp;%number of agents
ndim=3;
ti=0;
 %simulation step
t=ti:dt:tf;
nsteps=length(t);
gamma0=5;
lambda0=0.3;
%generate random ic
iniciales = get(handles.iniciales,'Value');
if iniciales == 0
    for i=1:np
        for kk=1:ndim
                x00(i,kk)=2*rand-1;
                v00(i,kk)=2*rand-1;
        end
    end
end

if iniciales == 1
    x00=[vectoresAgentes];
    v00=[velocidadAgentes];
end

vbar0=1/np*sum(v00,1);
%make initial vbar=0  comment next 3 lines for vbar!=0
v00(:,1)=-vbar0(1)+v00(:,1);
v00(:,2)=-vbar0(2)+v00(:,2);
v00(:,3)=-vbar0(3)+v00(:,3);
vbar2=1/np*sum(v00,1);


if iniciales == 0
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
end



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
 zdes = [-3 0 0; 3 -3 0;-3 0 0;1.5 1.5 -5;0.5 0.5 2];

 
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
 
if creativo == 1
    zdes = [vectoresAgentes];
end

%%

% recibido=get(handles.uitable10,'data');
% stringsolver = recibido(10,2);
% solver = eval(stringsolver{1});
if cargar == 1;
% x1 = load ('x1.mat')
% x2 = load ('x2.mat')
% x = load('x.mat') 
% x1 = load ('x1.txt')
% x2 = load ('x2.txt')
% x = load('x.txt') 
filename = 'solver.txt';
delimiterIn = ' ';
headerlinesIn = 1;
solver = importdata(filename,delimiterIn,headerlinesIn);
if solver == "RK4" || solver == "rk4"
% filename = 'x1.txt';
% delimiterIn = ' ';
% headerlinesIn = 1;
% x1 = importdata(filename,delimiterIn,headerlinesIn);
% filename = 'x2.txt';
% delimiterIn = ' ';
% headerlinesIn = 1;
% x2 = importdata(filename,delimiterIn,headerlinesIn);
% filename = 'x.txt';
% delimiterIn = ' ';
% headerlinesIn = 1;
% x = importdata(filename,delimiterIn,headerlinesIn);
cargarArchivotxt = get(handles.edit18,'String');
archivo1 = strcat(cargarArchivotxt,'-x1.txt')
archivo2 = strcat(cargarArchivotxt,'-x2.txt')
archivo3 = strcat(cargarArchivotxt,'-x3.txt')
archivo4 = strcat(cargarArchivotxt,'-v1.txt')
archivo5 = strcat(cargarArchivotxt,'-v2.txt')
archivo6 = strcat(cargarArchivotxt,'-v3.txt')
% solver = strcat(cargarArchivotxt,'.txt')
% filename = 'x1.txt';
delimiterIn = ' ';
headerlinesIn = 1;
x1 = importdata(archivo1,delimiterIn,headerlinesIn);
delimiterIn = ' ';
headerlinesIn = 1;
x2 = importdata(archivo2,delimiterIn,headerlinesIn);
delimiterIn = ' ';
headerlinesIn = 1;
x3 = importdata(archivo3,delimiterIn,headerlinesIn);
delimiterIn = ' ';
headerlinesIn = 1;
v1 = importdata(archivo4,delimiterIn,headerlinesIn);
delimiterIn = ' ';
headerlinesIn = 1;
v2 = importdata(archivo5,delimiterIn,headerlinesIn);
delimiterIn = ' ';
headerlinesIn = 1;
v3 = importdata(archivo6,delimiterIn,headerlinesIn);
delimiterIn = ' ';
headerlinesIn = 1;
solver = importdata('solver.txt',delimiterIn,headerlinesIn);
end

else

if solver == "ode23s"

        %ode23s%%%
        xinitt=[x00(:,1);v00(:,1);x00(:,2);v00(:,2);x00(:,3);v00(:,3)];
        [t,X]=ode23s(@csdynmatlab,0:dt:tf,xinitt);
        X=X';
        ftime=length(t);
        x1=X(1:np,:);
        v1=X(np+1:2*np,:); % desde np + 1 hasta 2np tenemos la velocidad en el ejex
        x2=X(2*np+1:3*np,:);
        v2=X(3*np+1:4*np,:);
        x3=X(4*np+1:5*np,:);
        v3=X(5*np+1:6*np,:);

end


if solver == "RK4" || solver== "rk4"
%RK
for i=1:nsteps-1
     
    %x(:,:,i+1)=RK4(x(:,:,i),u(:,:,i),dt,@cscdyn);
    x(:,:,i+1)=RK4(x(:,:,i),u(:,:,i),dt,@csdynshape);
    x0=squeeze(x(1:np,:,i+1));
    v0=squeeze(x(np+1:end,:,i+1));
    

    
end
end

toc
end
if grabar == 1
    palabra = get(handles.edit17,'String')
    if solver == "RK4" || solver== "rk4"
        palabra1 = strcat(palabra,'-x.txt')
        palabra2 = strcat(palabra,'-v.txt')        
        save(palabra1,'x0')
        save(palabra2,'v0')        
        save('solver.txt','solver')
    end
    if solver == "ode23s"
        palabra1 = strcat(palabra,'-x1.txt')
        palabra2 = strcat(palabra,'-x2.txt')
        palabra3 = strcat(palabra,'-x3.txt')
        palabra4 = strcat(palabra,'-v1.txt')
        palabra5 = strcat(palabra,'-v2.txt')
        palabra6 = strcat(palabra,'-v3.txt')

        save(palabra1,'x1')
        save(palabra2,'x2')
        save(palabra3,'x3')
        save(palabra4,'v1')
        save(palabra5,'v2')
        save(palabra6,'v3')
        save('solver.txt','solver')
    end
% save('x1.txt','x1')
% save('x2.txt','x2')
% save('x.txt','x')
end
grabar = 0;


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

%valores filas xx columnas yy dimension zz de la matriz z
if solver == "ode23s" 
    [xx,yy,zz] = size(x1);
    xx;
    yy;
    zz;
    yydividido = yy/79;
    if yy <= slider
        
        set(handles.slider2,'MIN',0,'MAX',yydividido);
        set(handles.slider7,'MIN',0,'MAX',yydividido);

    end
    
end

if solver == "RK4" || solver == "rk4"
    [xx,yy,zz] = size(x);
    xx;
    yy;
    zz;
    zzdividido=zz/79;
    if zz <= slider
        
        zzdividido=zz/79;
        set(handles.slider2,'MIN',0,'MAX',zzdividido);
        set(handles.slider7,'MIN',0,'MAX',zzdividido);

    end
end


%while3d
  while dd ==0

%Menu2 es para mostrar en una lista las posiciones de los agentes
menu2 = [1:np];
    set(handles.popupmenu6,'string',menu2) 
%Menu3 almacenara las posiciones espaciales de cada agente    
menu3=[];  
 menu3=zeros(np,3);
      
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



%figure

%  a=get(gca,'xlim');b=get(gca,'ylim');c=get(gca,'zlim');

 ejex=get(handles.ejey,'Value')/10;
 ejey=get(handles.ejex,'Value')/10;
 zoom=get(handles.slider10,'Value')/10;
 cabeza_agente = get(handles.cabeza,'String');
 cabeza_agente2 = get(handles.cabezano,'String');

  grid on

     
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

if cabeza_agente == "Con cabeza"
set(handles.cabeza,'visible','on'); 
set(handles.cabezano,'visible','off');
set (handles.pushbutton16, 'BackgroundColor' , 'green' ) 
a=[-zoom+ejey zoom+ejey];b=[-zoom zoom];c=[-zoom+ejex zoom+ejex];
     axis([a b c])
for kk=1:np
     
  %comment for removing trajectories
  zoom_agente= get(handles.pushbutton6,'String');
  o = get(handles.popupmenu3,'Value');% recibe el valor del menu de agente
  front = get(handles.radiobutton11,'Value');
  back = get(handles.radiobutton12,'Value');
  estela = get(handles.estela,'Value');
  
 %%%%% CON SOLVER RK4 PARA PLOT  %%%%%%
if solver == "RK4" | solver == "rk4"
         if estela == 0
             plot3(squeeze(x(kk,1,ss2:1:ss)),squeeze(x(kk,2,ss2:1:ss)),squeeze(x(kk,3,ss2:1:ss)),'Color',col(kk,:),'LineWidth',2','LineStyle','-')
       
         end
        
        if estela == 1
            plot3(squeeze(x(kk,1,ss2:1:ss)),squeeze(x(kk,2,ss2:1:ss)),squeeze(x(kk,3,ss2:1:ss)),'Color',col(kk,:),'LineWidth',2','LineStyle','-')
            if ss > 10
                
                avanzando = ss-7;
                set(handles.slider7,'Value',avanzando);
            end
            if ss <10
                set(handles.slider7,'Value',0);
            end
            %             if ss > 100
%                   plot3(squeeze(x(kk,1,ss2-100:50:ss)),squeeze(x(kk,2,ss-100:50:ss)),squeeze(x(kk,3,ss-100:50:ss)),'Color',col(kk,:),'LineWidth',2','LineStyle','-')
%             else
%                 
%                 if ss<20
%                     plot3(squeeze(x(kk,1,ss2:1:ss)),squeeze(x(kk,2,ss2:1:ss)),squeeze(x(kk,3,ss2:1:ss)),'Color',col(kk,:),'LineWidth',2','LineStyle','-')
%                 end
%                 if ss > 20
%                     plot3(squeeze(x(kk,1,ss-20:10:ss)),squeeze(x(kk,2,ss-20:10:ss)),squeeze(x(kk,3,ss-20:10:ss)),'Color',col(kk,:),'LineWidth',2','LineStyle','-')
%                 end
%             end
            
        
        end 
        menu3(kk,:)=[x(kk,1,ss) x(kk,2,ss) x(kk,3,ss)];
end


%%%%% CON SOLVER ode23s PARA PLOT  %%%%

if solver == "ode23s"
         if estela == 0
            plot3(squeeze(x1(kk,ss2:1:ss)),squeeze(x2(kk,ss2:1:ss)),squeeze(x3(kk,ss2:1:ss)),'Color',col(kk,:),'LineWidth',2','LineStyle','-')
         end
        
        if estela == 1
            
            plot3(squeeze(x1(kk,ss2:1:ss)),squeeze(x2(kk,ss2:1:ss)),squeeze(x3(kk,ss2:1:ss)),'Color',col(kk,:),'LineWidth',2','LineStyle','-')
            if ss > 10
                
                avanzando = ss-7;
                set(handles.slider7,'Value',avanzando);
            end
            if ss <10
                set(handles.slider7,'Value',0);
            end
%             if ss > 100
%                   plot3((x1(kk,ss-100:50:ss)),(x2(kk,ss-100:50:ss)),(x3(kk,ss-100:50:ss)),'Color',col(kk,:),'LineWidth',2','LineStyle','-')
%             else
%                 
%                 if ss<20
%                     plot3((x1(kk,ss2:1:ss)),(x2(kk,ss2:1:ss)),(x3(kk,ss2:1:ss)),'Color',col(kk,:),'LineWidth',2','LineStyle','-')
%                 end
%                 if ss > 20
%                     plot3((x1(kk,ss-20:10:ss)),(x2(kk,ss-20:10:ss)),(x3(kk,ss-20:10:ss)),'Color',col(kk,:),'LineWidth',2','LineStyle','-')
%                 end
%             end
            
        
        end 
        menu3(kk,:)=[x1(kk,ss) x2(kk,ss) x3(kk,ss)];
end
  
    
    v=get(handles.popupmenu6,'Value');
    set(handles.text22,'string',"x = "+(menu3(v,1))+";  "+"y = "+(menu3(v,2))+";  "+"z = "+(menu3(v,3)));
  
if zoom_agente == "Zoom Personal" & front == 1%% controla la cabeza del agente para realizar zoom de seguimiento
       
     if solver == "RK4" | solver == "rk4"
     a=[x(o,2,ss)-zoom+ejey x(o,2,ss)+zoom+ejey];b=[-zoom+x(o,3,ss) x(o,3,ss)+zoom];c=[x(o,1,ss)-zoom+ejex x(o,1,ss)+zoom+ejex];
     axis([c a b])
     end
     
     if solver == "ode23s"
     a=[x2(o,ss)-zoom+ejey x2(o,ss)+zoom+ejey];b=[-zoom+x3(o,ss) x3(o,ss)+zoom];c=[x1(o,ss)-zoom+ejex x1(o,ss)+zoom+ejex];
     axis([c a b])
     end
     
elseif zoom_agente == "Zoom Personal" & back == 1
%           set(handles.salir,'visible','on');
%           set(handles.salir,'value',0); 
       a=[x(o,2,ss2)-zoom+ejey x(o,2,ss2)+zoom+ejey];b=[-zoom+x(o,3,ss2) x(o,3,ss2)+zoom];c=[x(o,1,ss2)-zoom+ejex x(o,1,ss2)+zoom+ejex];
       axis([c a b])
       
       if solver == "RK4" | solver == "rk4"
            a=[x(o,2,ss2)-zoom+ejey x(o,2,ss2)+zoom+ejey];b=[-zoom+x(o,3,ss2) x(o,3,ss2)+zoom];c=[x(o,1,ss2)-zoom+ejex x(o,1,ss2)+zoom+ejex];
            axis([c a b])
       end
     
       if solver == "ode23s"
            a=[x2(o,ss2)-zoom+ejey x2(o,ss2)+zoom+ejey];b=[-zoom+x3(o,ss2) x3(o,ss2)+zoom];c=[x1(o,ss2)-zoom+ejex x1(o,ss2)+zoom+ejex];
            axis([c a b])
       end
       
            
            
else
 salirzoom=get(handles.salirzoom,'String');
 if salirzoom == "Salir zoom"
  a=[-zoom+ejey zoom+ejey];b=[-zoom zoom];c=[-zoom+ejex zoom+ejex];
  axis([a b c])
 end   
  end

end
end
if cabeza_agente == "ok"
set(handles.cabeza,'visible','off'); 
set(handles.cabezano,'visible','on');
  zoom_agente= get(handles.pushbutton6,'String');
  o = get(handles.popupmenu3,'Value');% recibe el valor del menu de agente
  front = get(handles.radiobutton11,'Value');
  back = get(handles.radiobutton12,'Value');
  estela = get(handles.estela,'Value');
% Menu2 es para mostrar en una lista las posiciones de los agentes
% menu2 = [1:np]
%     set(handles.popupmenu6,'string',menu2) 
% Menu3 almacenara las posiciones espaciales de cada agente    
% menu3=[];


a=[-zoom+ejey zoom+ejey];b=[-zoom zoom];c=[-zoom+ejex zoom+ejex];
  axis([a b c])

 for  kk=1:np 
     
 

if solver == "RK4" | solver =="rk4"
        if estela == 1
            
            plot3(squeeze(x(kk,1,ss2:1:ss)),squeeze(x(kk,2,ss2:1:ss)),squeeze(x(kk,3,ss2:1:ss)),'Color',col(kk,:),'LineWidth',2','LineStyle','-')
            if ss > 10
                
                avanzando = ss-7;
                set(handles.slider7,'Value',avanzando);
            end
            if ss <10
                set(handles.slider7,'Value',0);
            end
            
            
%             if ss > 100
%                 plot3(squeeze(x(kk,1,ss-100:50:ss)),squeeze(x(kk,2,ss-100:50:ss)),squeeze(x(kk,3,ss-100:50:ss)),'Color',col(kk,:),'LineWidth',2','LineStyle','-')
%             else
%                 
%                 if ss<20
%                     plot3(squeeze(x(kk,1,ss2:1:ss)),squeeze(x(kk,2,ss2:1:ss)),squeeze(x(kk,3,ss2:1:ss)),'Color',col(kk,:),'LineWidth',2','LineStyle','-')
%                 end
%                 if ss > 20
%                    
%                     plot3(squeeze(x(kk,1,ss-20:1:ss)),squeeze(x(kk,2,ss-20:1:ss)),squeeze(x(kk,3,ss-20:1:ss)),'Color',col(kk,:),'LineWidth',2','LineStyle','-')
%                 end
% 
%             end
            
%             plot(x1(i,ss),x2(i,ss),'Marker','o','MarkerSize',10,'MarkerFaceColor',col(i,:))
        end
        
    plot3(squeeze(x(kk,1,ss)),squeeze(x(kk,2,ss)),squeeze(x(kk,3,ss)),'Marker','o','MarkerSize',10,'MarkerFaceColor',col(kk,:))
    
    menu3(kk,:)=[x(kk,1,ss) x(kk,2,ss) x(kk,3,ss)];
   
    
    
end 

%%%%%    CON ode23s   %%%%%%
if solver == "ode23s"
    
    if estela == 0
        
       plot3(squeeze(x1(kk,ss2:1:ss)),squeeze(x2(kk,ss2:1:ss)),squeeze(x3(kk,ss2:1:ss)),'Color',col(kk,:),'LineWidth',2','LineStyle','-')
    end
    if estela == 1

            plot3(squeeze(x1(kk,ss2:1:ss)),squeeze(x2(kk,ss2:1:ss)),squeeze(x3(kk,ss2:1:ss)),'Color',col(kk,:),'LineWidth',2','LineStyle','-')
            if ss > 10
                
                avanzando = ss-7;
                set(handles.slider7,'Value',avanzando);
            end
            if ss <10
                set(handles.slider7,'Value',0);
            end
%             if ss > 100
%                 plot3((x1(kk,ss-100:50:ss)),(x2(kk,ss-100:50:ss)),(x3(kk,ss-100:50:ss)),'Color',col(kk,:),'LineWidth',2','LineStyle','-')
%             else
%                 
%                 if ss<20
%                     plot3((x1(kk,ss2:1:ss)),(x2(kk,ss2:1:ss)),(x3(kk,ss2:1:ss)),'Color',col(kk,:),'LineWidth',2','LineStyle','-')
%                 end
%                 if ss > 20
%                    
%                     plot3((x1(kk,ss-20:1:ss)),(x2(kk,ss-20:1:ss)),(x3(kk,ss-20:1:ss)),'Color',col(kk,:),'LineWidth',2','LineStyle','-')
%                 end
% 
%             end
            
%             plot(x1(i,ss),x2(i,ss),'Marker','o','MarkerSize',10,'MarkerFaceColor',col(i,:))
        end
    plot3((x1(kk,ss)),squeeze(x2(kk,ss)),squeeze(x3(kk,ss)),'Marker','o','MarkerSize',10,'MarkerFaceColor',col(kk,:))
    menu3(kk,:)=[x1(kk,ss) x2(kk,ss) x3(kk,ss)];
end

if zoom_agente == "Zoom Personal" & front == 1
   if solver == "RK4" | solver == "rk4"
     a=[x(o,2,ss)-zoom+ejey x(o,2,ss)+zoom+ejey];b=[-zoom+x(o,3,ss) x(o,3,ss)+zoom];c=[x(o,1,ss)-zoom+ejex x(o,1,ss)+zoom+ejex];
     axis([c a b])
     end
     
     if solver == "ode23s"
     a=[x2(o,ss)-zoom+ejey x2(o,ss)+zoom+ejey];b=[-zoom+x3(o,ss) x3(o,ss)+zoom];c=[x1(o,ss)-zoom+ejex x1(o,ss)+zoom+ejex];
     axis([c a b])
     end 
      
  end 
  
  if zoom_agente == "Zoom Personal" & back == 1
   if solver == "RK4" | solver == "rk4"
     a=[x(o,2,ss2)-zoom+ejey x(o,2,ss2)+zoom+ejey];b=[-zoom+x(o,3,ss2) x(o,3,ss2)+zoom];c=[x(o,1,ss2)-zoom+ejex x(o,1,ss2)+zoom+ejex];
     axis([c a b])
     end
     
     if solver == "ode23s"
     a=[x2(o,ss2)-zoom+ejey x2(o,ss2)+zoom+ejey];b=[-zoom+x3(o,ss2) x3(o,ss2)+zoom];c=[x1(o,ss2)-zoom+ejex x1(o,ss2)+zoom+ejex];
     axis([c a b])
     end 
      
  end 
  
  if salirzoom == "Salir zoom"
    a=[-zoom+ejey zoom+ejey];b=[-zoom zoom];c=[-zoom+ejex zoom+ejex];
    axis([a b c])
  end
  
% menu3(kk,:)=[x(kk,1,ss) x(kk,2,ss) x(kk,3,ss)];
 end
v=get(handles.popupmenu6,'Value');
set(handles.text22,'string',"x = "+(menu3(v,1))+";  "+"y = "+(menu3(v,2))+";  "+"z = "+(menu3(v,3)));

end
pause(0.01)
 
 
 
  view(-view11,view22)
 
sss=ss;
view2=view1;    
menu = [1:np]; 
    set(handles.popupmenu3,'string',menu)
end    
set(handles.popupmenu6,'Visible','off')
set(handles.text22,'Visible','off')
% end
cla reset
end
%******** FIN CODIGO 3D *****************
end
end



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
global cuadrado bandada poligon vector vectoresAgentes vector2 velocidadAgentes;
vectoresAgentes = [];
vector = 1;
velocidadAgentes = [];
vector2 = 1;
refreshOn = 1;
set(handles.text28,'Visible','off')
set(handles.edit12,'Visible','off')
set(handles.edit13,'Visible','off')
set(handles.edit14,'Visible','off')
set(handles.text25,'Visible','off')
set(handles.text26,'Visible','off')
set(handles.text27,'Visible','off')

set(handles.text29,'Visible','off')
set(handles.edit19,'Visible','off')
set(handles.edit20,'Visible','off')
set(handles.edit21,'Visible','off')
set(handles.text30,'Visible','off')
set(handles.text31,'Visible','off')
set(handles.text32,'Visible','off')

set(handles.text13,'visible','off')
set(handles.cabeza,'visible','off'); 
set(handles.cabezano,'visible','off');
set(handles.pushbutton6,'visible','on');
set(handles.uitable10,'Visible','off')
set(handles.pushbutton17,'Visible','off')
set(handles.iniciales,'Value',0)
set(handles.slider15,'Visible','off');
set(handles.slider16,'Visible','off');
set(handles.slider16,'Visible','off');
set(handles.salirzoom,'Visible','off');
set(handles.pushbutton9,'visible','off')
set(handles.uibuttongroup3,'visible','off')
% set(handles.salir,'visible','off');
set(handles.popupmenu3,'visible','off')

set(handles.text22,'String',' ')
set(handles.edit1,'String',' ')
set(handles.edit12,'String',' ')
set(handles.edit13,'String',' ')
set(handles.edit14,'String',' ')
set(handles.edit19,'String',' ')
set(handles.edit20,'String',' ')
set(handles.edit21,'String',' ')
set(handles.text25,'String',' ')
set(handles.text26,'String',' ')
set(handles.text27,'String',' ')
set(handles.text28,'String',' ')
set(handles.text29,'String',' ')
set(handles.text30,'String',' ')
set(handles.text31,'String',' ')
set(handles.text32,'String',' ')
set(handles.stop,'String','Stop.')
set(handles.cabeza,'String','Cabeza');
set(handles.popupmenu6,'String','Agent position')
set(handles.text8,'String','Ingrese nuevos valores')
set(handles.pushbutton6,'String',"Stop")
set(handles.pushbutton5,'String',"Stop")
set(handles.salirzoom,'String','Stop')

set(handles.original,'Value',1);
set(handles.slider2,'Value',0);
set(handles.slider7,'Value',0);
set(handles.pushbutton6,'Value',0)
set(handles.pushbutton5,'Value',0)

set (handles.pushbutton16, 'BackgroundColor' , 'red' ) 
 set(handles.uitable10,'Visible','off')
 set(handles.pull,'String','Refresh')

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
winopen('Nosotros.docx');
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
set(handles.pushbutton6,'visible','off')
set(handles.salirzoom,'visible','on')
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
global np ndim sigma K beta pp flag alpha M zdes dt refreshOn frontRefresh ejeyRefresh ejexRefresh zoomRefresh
dos=get(handles.radiobutton26,'Value');
tres=get(handles.radiobutton27,'Value');
refresh = get(handles.pull,'String');
A = get(handles.uitable10,'data');
vectores = size(A);

%%borre esto del if refresh
%%|| vectores(1,1)

if refresh == "Push" 
if dos==1
    
    set(handles.uitable10,'Visible','on');
    A = {'Alpha';'Beta';'Sigma';'K';'PP';'Flag';'M';'Tiempo Final';'Dif. tiempo'; 'Solver'; 'Influence';'Influenceu';'Limit Front Control';'y';'x';'zoom'};
    B = {'1.1';'0.4';'0';'100';'2';'0';'50';'20';'.001';'ode23s';'1./((0+distance).^pow)';'1./((1+distance.^2).^pow)';frontRefresh;ejeyRefresh;ejexRefresh;zoomRefresh};
    variables = [A B];
    set(handles.uitable10,'data',variables);
end

if tres == 1
    set(handles.uitable10,'Visible','on');
    A = {'Alpha';'Beta';'Sigma';'K';'PP';'Flag';'M';'Tiempo Final';'Dif. tiempo';'Solver'; 'Influence';'Influenceu';'Limit Front Control';'y';'x';'zoom'};
    B = {'1.1';'0.1';'0';'10';'4';'0';'50';'400';'0.05';'RK4';'1./((0+distance).^pow)';'1./((1+distance.^2).^pow)';frontRefresh;ejeyRefresh;ejexRefresh;zoomRefresh};
    variables = [A B];
    set(handles.uitable10,'data',variables);    
end

set(handles.pull,'String','Refresh')

else
        
if dos==1
    set(handles.uitable10,'Visible','on');
    A = {'Alpha';'Beta';'Sigma';'K';'PP';'Flag';'M';'Tiempo Final';'Dif. tiempo'; 'Solver'; 'Influence';'Influenceu';'Limit Front Control';'y';'x';'zoom'};
    B = {'1.1';'0.4';'0';'100';'2';'0';'50';'20';'.001';'ode23s';'1./((0+distance).^pow)';'1./((1+distance.^2).^pow)';'20001';'100';'100';'100' };
    variables = [A B];
    set(handles.uitable10,'data',variables);
end

if tres == 1
    set(handles.uitable10,'Visible','on');
    A = {'Alpha';'Beta';'Sigma';'K';'PP';'Flag';'M';'Tiempo Final';'Dif. tiempo';'Solver'; 'Influence';'Influenceu';'Limit Front Control';'y';'x';'zoom'};
    B = {'1.1';'0.1';'0';'10';'4';'0';'50';'400';'0.05';'RK4';'1./((0+distance).^pow)';'1./((1+distance.^2).^pow)';'101';'100';'100';'100'};
    variables = [A B];
    set(handles.uitable10,'data',variables);
end
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
global cuadrado bandada poligon elegir circulo triangulo creativo grabar cargar cargarzdes
elegir = 0;
cuadrado = 0;
bandada = 0;
poligon = 0;
circulo = 0;
triangulo = 0;
creativo = 0;
cargar = 0;
cargarzdes = 0;
v=get(handles.popupmenu4,'Value');
%bandada = get(handles.popupmenu4,'Value');

switch v
    
        
    case 2
        cuadrado = 1;
        set(handles.edit12,'Visible','off')
        set(handles.edit13,'Visible','off')
        set(handles.edit14,'Visible','off')
        set(handles.text25,'Visible','off')
        set(handles.text26,'Visible','off')
        set(handles.text27,'Visible','off')
        set(handles.text28,'Visible','off')
        set(handles.pushbutton17,'Visible','off')
        set(handles.pushbutton30,'Visible','on')
        set(handles.pushbutton31,'Visible','off')
        set(handles.edit17,'Visible','on')
        set(handles.edit18,'Visible','off')
    case 3
        bandada = 1;
        set(handles.edit12,'Visible','off')
        set(handles.edit13,'Visible','off')
        set(handles.edit14,'Visible','off')
        set(handles.text25,'Visible','off')
        set(handles.text26,'Visible','off')
        set(handles.text27,'Visible','off')
        set(handles.text28,'Visible','off')
        set(handles.pushbutton17,'Visible','off')
        set(handles.pushbutton30,'Visible','on')
        set(handles.pushbutton31,'Visible','off')
        set(handles.edit17,'Visible','on')
        set(handles.edit18,'Visible','off')
    case 4
        poligon = 1;
        set(handles.edit12,'Visible','off')
        set(handles.edit13,'Visible','off')
        set(handles.edit14,'Visible','off')
        set(handles.text25,'Visible','off')
        set(handles.text26,'Visible','off')
        set(handles.text27,'Visible','off')
        set(handles.text28,'Visible','off')
        set(handles.pushbutton17,'Visible','off')
        set(handles.pushbutton30,'Visible','on')
        set(handles.pushbutton31,'Visible','off')
        set(handles.edit17,'Visible','on')
        set(handles.edit18,'Visible','off')
    case 5
        circulo = 1;
        set(handles.edit12,'Visible','off')
        set(handles.edit13,'Visible','off')
        set(handles.edit14,'Visible','off')
        set(handles.text25,'Visible','off')
        set(handles.text26,'Visible','off')
        set(handles.text27,'Visible','off')
        set(handles.text28,'Visible','off')
        set(handles.pushbutton17,'Visible','off')
        set(handles.pushbutton30,'Visible','on')
        set(handles.pushbutton31,'Visible','off')
        set(handles.edit17,'Visible','on')
        set(handles.edit18,'Visible','off')
    case 6
        triangulo = 1;
        set(handles.edit12,'Visible','off')
        set(handles.edit13,'Visible','off')
        set(handles.edit14,'Visible','off')
        set(handles.text25,'Visible','off')
        set(handles.text26,'Visible','off')
        set(handles.text27,'Visible','off')
        set(handles.text28,'Visible','off')
        set(handles.pushbutton17,'Visible','off')
        set(handles.pushbutton30,'Visible','on')
        set(handles.pushbutton31,'Visible','off')
        set(handles.edit17,'Visible','on')
        set(handles.edit18,'Visible','off')
    case 7
        creativo = 1;    
        set(handles.text28,'Visible','on')
        set(handles.edit12,'Visible','on')
        set(handles.edit13,'Visible','on')
        set(handles.edit14,'Visible','off')
        set(handles.text25,'Visible','on')
        set(handles.text26,'Visible','on')
        set(handles.text25,'String','x')
        set(handles.text26,'String','y')
        set(handles.text27,'Visible','off')
        set(handles.text28,'String','Ingrese Vectores')
        set(handles.pushbutton17,'Visible','on')
        set(handles.pushbutton30,'Visible','on')
        set(handles.pushbutton31,'Visible','off')
        set(handles.edit17,'Visible','on')
        set(handles.edit18,'Visible','off')
    case 8
        cargarzdes = 1;
        set(handles.edit12,'Visible','off')
        set(handles.edit13,'Visible','off')
        set(handles.edit14,'Visible','off')
        set(handles.text25,'Visible','off')
        set(handles.text26,'Visible','off')
        set(handles.text27,'Visible','off')
        set(handles.text28,'Visible','off')
        set(handles.pushbutton17,'Visible','off')
        set(handles.pushbutton30,'Visible','on')
        set(handles.pushbutton31,'Visible','off')
        set(handles.edit17,'Visible','on')
        set(handles.edit18,'Visible','off')
    case 9
        cargar  = 1;
        set(handles.edit12,'Visible','off')
        set(handles.edit13,'Visible','off')
        set(handles.edit14,'Visible','off')
        set(handles.text25,'Visible','off')
        set(handles.text26,'Visible','off')
        set(handles.text27,'Visible','off')
        set(handles.text28,'Visible','off')
        set(handles.pushbutton17,'Visible','off')
        set(handles.pushbutton30,'Visible','off')
        set(handles.pushbutton31,'Visible','on')
        set(handles.edit17,'Visible','off')
        set(handles.edit18,'Visible','on')
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
global cuadrado paredes circulo elegir bandada poligon olympic piramide creativo grabar cargar cargarzdes
elegir = 0;
cuadrado = 0;
paredes = 0;
circulo = 0;
bandada = 0;
olympic = 0;
piramide = 0;
creativo = 0;
cargar = 0;
cargarzdes = 0;
v=get(handles.popupmenu7,'Value');
%bandada = get(handles.popupmenu4,'Value');

switch v
    
        
    case 2
        cuadrado = 1;
        set(handles.edit12,'Visible','off')
        set(handles.edit13,'Visible','off')
        set(handles.edit14,'Visible','off')
        set(handles.text25,'Visible','off')
        set(handles.text26,'Visible','off')
        set(handles.text27,'Visible','off')
        set(handles.text28,'Visible','off')
        set(handles.pushbutton30,'Visible','on')
        set(handles.pushbutton31,'Visible','off')
        set(handles.edit17,'Visible','on')
        set(handles.edit18,'Visible','off')
    case 3
        paredes = 1;
        set(handles.edit12,'Visible','off')
        set(handles.edit13,'Visible','off')
        set(handles.edit14,'Visible','off')
        set(handles.text25,'Visible','off')
        set(handles.text26,'Visible','off')
        set(handles.text27,'Visible','off')
        set(handles.text28,'Visible','off')
        set(handles.pushbutton30,'Visible','on')
        set(handles.pushbutton31,'Visible','off')
        set(handles.edit17,'Visible','on')
        set(handles.edit18,'Visible','off')
    case 4
        circulo = 1;
        set(handles.edit12,'Visible','off')
        set(handles.edit13,'Visible','off')
        set(handles.edit14,'Visible','off')
        set(handles.text25,'Visible','off')
        set(handles.text26,'Visible','off')
        set(handles.text27,'Visible','off')
        set(handles.text28,'Visible','off')
        set(handles.pushbutton30,'Visible','on')
        set(handles.pushbutton31,'Visible','off')
        set(handles.edit17,'Visible','on')
        set(handles.edit18,'Visible','off')
    case 5
        piramide=1;
        set(handles.edit12,'Visible','off')
        set(handles.edit13,'Visible','off')
        set(handles.edit14,'Visible','off')
        set(handles.text25,'Visible','off')
        set(handles.text26,'Visible','off')
        set(handles.text27,'Visible','off')
        set(handles.text28,'Visible','off')
        set(handles.pushbutton30,'Visible','on')
        set(handles.pushbutton31,'Visible','off')
        set(handles.edit17,'Visible','on')
        set(handles.edit18,'Visible','off')
    case 6
        bandada=1;
        set(handles.edit12,'Visible','off')
        set(handles.edit13,'Visible','off')
        set(handles.edit14,'Visible','off')
        set(handles.text25,'Visible','off')
        set(handles.text26,'Visible','off')
        set(handles.text27,'Visible','off')
        set(handles.text28,'Visible','off')
        set(handles.pushbutton30,'Visible','on')
        set(handles.pushbutton31,'Visible','off')
        set(handles.edit17,'Visible','on')
        set(handles.edit18,'Visible','off')
    case 7   
        poligon=1;
        set(handles.edit12,'Visible','off')
        set(handles.edit13,'Visible','off')
        set(handles.edit14,'Visible','off')
        set(handles.text25,'Visible','off')
        set(handles.text26,'Visible','off')
        set(handles.text27,'Visible','off')
        set(handles.text28,'Visible','off')
        set(handles.pushbutton30,'Visible','on')
        set(handles.pushbutton31,'Visible','off')
        set(handles.edit17,'Visible','on')
        set(handles.edit18,'Visible','off')
    case 8
        olympic=1;
        set(handles.edit12,'Visible','off')
        set(handles.edit13,'Visible','off')
        set(handles.edit14,'Visible','off')
        set(handles.text25,'Visible','off')
        set(handles.text26,'Visible','off')
        set(handles.text27,'Visible','off')
        set(handles.text28,'Visible','off')
        set(handles.pushbutton30,'Visible','on')
        set(handles.pushbutton31,'Visible','off')
        set(handles.edit17,'Visible','on')
        set(handles.edit18,'Visible','off')
    case 9
        creativo=1;    
        set(handles.text28,'Visible','on')
        set(handles.edit12,'Visible','on')
        set(handles.edit13,'Visible','on')
        set(handles.edit14,'Visible','on')
        set(handles.text25,'Visible','on')
        set(handles.text26,'Visible','on')
        set(handles.text27,'Visible','on')
        set(handles.text25,'String','x')
        set(handles.text26,'String','y')
        set(handles.text27,'String','z')
        set(handles.text28,'String','Ingrese Vectores')
        set(handles.pushbutton30,'Visible','on')
        set(handles.pushbutton31,'Visible','off')
        set(handles.edit17,'Visible','on')
        set(handles.edit18,'Visible','off')
    case 10
        cargarzdes = 1;
        set(handles.text28,'Visible','on')
        set(handles.edit12,'Visible','on')
        set(handles.edit13,'Visible','on')
        set(handles.edit14,'Visible','on')
        set(handles.text25,'Visible','on')
        set(handles.text26,'Visible','on')
        set(handles.text27,'Visible','on')
        set(handles.text25,'String','x')
        set(handles.text26,'String','y')
        set(handles.text27,'String','z')
        set(handles.text28,'String','Ingrese Vectores')
        set(handles.pushbutton30,'Visible','on')
        set(handles.pushbutton31,'Visible','off')
        set(handles.edit17,'Visible','on')
        set(handles.edit18,'Visible','off')
    case 11
        cargar = 1;
        set(handles.edit12,'Visible','off')
        set(handles.edit13,'Visible','off')
        set(handles.edit14,'Visible','off')
        set(handles.text25,'Visible','off')
        set(handles.text26,'Visible','off')
        set(handles.text27,'Visible','off')
        set(handles.text28,'Visible','off')
        set(handles.pushbutton30,'Visible','off')
        set(handles.pushbutton31,'Visible','on')
        set(handles.edit17,'Visible','off')
        set(handles.edit18,'Visible','on')
        
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
set(handles.iniciales,'Value',0);

%ocultar menu de ingreso de x y v iniciales cunado se cambia de dim

set(handles.text28,'String','Ingrese Vectores')
    set(handles.text28,'Visible','off')
    set(handles.text25,'Visible','off')
    set(handles.text26,'Visible','off')
    set(handles.text29,'Visible','off')
    set(handles.text30,'Visible','off')
    set(handles.text31,'Visible','off')
    set(handles.pushbutton17,'Visible','off')
    set(handles.edit12,'Visible','off')
    set(handles.edit13,'Visible','off')
    set(handles.edit19,'Visible','off')
    set(handles.edit20,'Visible','off')
    set(handles.edit21,'Visible','off')
        set(handles.edit14,'Visible','off')
        set(handles.text27,'Visible','off')
        set(handles.text32,'Visible','off')

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


set(handles.iniciales,'Value',0);
%si se cambia de dim, se oculte el menu de ingreso de x y v iniciales
set(handles.text28,'String','Ingrese Vectores')
    set(handles.text28,'Visible','off')
    set(handles.text25,'Visible','off')
    set(handles.text26,'Visible','off')
    set(handles.text29,'Visible','off')
    set(handles.text30,'Visible','off')
    set(handles.text31,'Visible','off')
    set(handles.pushbutton17,'Visible','off')
    set(handles.edit12,'Visible','off')
    set(handles.edit13,'Visible','off')
    set(handles.edit19,'Visible','off')
    set(handles.edit20,'Visible','off')
    set(handles.edit21,'Visible','off')
        set(handles.edit14,'Visible','off')
        set(handles.text27,'Visible','off')
        set(handles.text32,'Visible','off')

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
global np ndim beta pp flag alpha zdes M K voldvi xnodevi aoldxvi aoldyvi An str1
global deseado original str2 distance pow str3 str4 sigma

%influenceu=@(distance,pow)distance^5./((1+distance.^2).^pow); 
%influenceu=@(distance,pow)0;

if original == 1
   influenceu = str1; 
   influence = str2;
end

if deseado == 1
   influenceu=str1;
   influence = @(distance,pow)eval(str2{1});
end
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
global np ndim beta pp flag distance pow alpha str4 zdes M K sigma str1 original deseado str2

if original == 1
   influenceu = str1; 
   influence = str2;
end

if deseado == 1
   influenceu=str1;
%    influence = @(distance,pow)eval(str2{1});
    influence = str4;
end

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
% function [x]=csdynshape3d(xinit,u)
% global np ndim beta pp flag alpha zdes M K 
% influenceu=@(distance,pow)1./((1+distance.^2).^pow); 
% flag2=0;
% if u==1
%     flag2=1;
% end
% u=zeros(np,ndim);
% 
% 
% 
% a=zeros(np,np,ndim);
% f=zeros(np,np,ndim);
% distance=zeros(np,np);
% difv=zeros(np,np,ndim);
% difx=zeros(np,np,ndim);
% Np=np;
% x0=xinit(1:np,:);
% v0=xinit(np+1:2*np,:);
% mdiag=-eye(np);
% front=[[zeros(1,np-1);eye(np-1)] zeros(np,1)];
% back=[zeros(np,1) [eye(np-1);zeros(1,np-1)]];
% 
% for i=1:np
%     distance(i,:)=sqrt(sum((repmat(x0(i,:),Np,1)-x0).^2,2));
%     difv(i,:,:)=v0-repmat(v0(i,:),Np,1);
%     difx(i,:,:)=-(x0-repmat(x0(i,:),Np,1));
% end
% distform=[];
% for i=1:np-1
%     %distance from desired formation
%     distform(i)=norm(x0(i,:)-x0(i+1,:)-zdes(i,:),2);
% end
% 
% distance=distance+eye(length(distance)); %this is to avoid division by zero
% alignment=1*sqrt((0.5/np)*(sum(sum(difv(:,:,1).^2))+sum(sum(difv(:,:,2).^2))));
% for k=1:ndim;
%     a(:,:,k)=K*influence(distance,alpha).*difv(:,:,k);
%     f(:,:,k)=repulsion(distance,pp).*difx(:,:,k);
% end
% 
% u(1,:)=-M*influenceu(distform(1),beta)*(x0(1,:)-x0(2,:)-zdes(1,:));
% u(Np,:)=M*influenceu(distform(Np-1),beta)*(x0(Np-1,:)-x0(Np,:)-zdes(Np-1,:));
% for i=2:Np-1
%     u(i,:)=M*influenceu(distform(i-1),beta)*(x0(i-1,:)-x0(i,:)-zdes(i-1,:))-M*influenceu(distform(i),beta)*(x0(i,:)-x0(i+1,:)-zdes(i,:));
% end
% 
% %zdes=[z1 z2];
% %     temp=diag(distform,1);
% %     temp2=diag(distform,-1);
% %     distform=distform-diag(temp,1)+diag(temp2,1);
% %     for k=1:ndim;
% %         au(:,:,k)=influenceu(distform,beta).*(-difx(:,:,k)+(back-front)*zdes(:,k)).*(back+front);
% %     end
% %     u=squeeze(sum(au,2));
% 
% % (1/np)*
% if flag2
%     u=u+[50 0;zeros(np-1,2)];
% end
% x(1:np,:)=v0;
% x(np+1:2*np,:)=squeeze(sum(a,2))./np+0*alignment*squeeze(sum(f,2))+1*u;
% %end

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
 
% function [a]=influenceu(distance,pow)
% recibido=get(handles.uitable10,'data');
%valores de la tabla
%     v2 = recibido(12,2);
%     influenceuu = str2double(v2);
%    a=1./((1+distance.^2).^pow);
%     a=influenceuu;
% 
%  end

%**************** FIN INFLUENCEU3D*********************************
% function [a]=influence(distance,pow)
% global sigma 
% if distance==0
%     a=0;
% else
%     a=1./((sigma+distance).^pow);
% 
% end

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



function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double


% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in estela.
function estela_Callback(hObject, eventdata, handles)
% hObject    handle to estela (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of estela


% --- Executes on button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vectoresAgentes  vector velocidadAgentes vector2;
iniciales = get(handles.iniciales,'Value');
vectorx = get(handles.edit12,'String');
vectory = get(handles.edit13,'String');
vectorz = get(handles.edit14,'String');

cont = 3;
dim = get(handles.radiobutton26,'Value');
if dim == 1
    cont = 2;
end

vectorx
vectory
vectorz

vectoresAgentes (vector,1)= [str2double(vectorx)];
set(handles.text25,'String',vectoresAgentes(vector,1));
set(handles.pushbutton17,'Value',0);

vectoresAgentes (vector,2)= [str2double(vectory)];
set(handles.text26,'String',vectoresAgentes(vector,2));
set(handles.pushbutton17,'Value',0);

if cont == 3
vectoresAgentes (vector,3)= [str2double(vectorz)];
set(handles.text27,'String',vectoresAgentes(vector,3));
set(handles.pushbutton17,'Value',0);
end 
vectoresAgentes
vector ;
set(handles.text28,'String',"Ingresado Vector Xi "+vector)

vector=vector+1;


%%%%%%%%%velocidades

if iniciales == 1
velocidadx = get(handles.edit19,'String');
velocidady = get(handles.edit20,'String');
velocidadz = get(handles.edit21,'String');

contv1 = 3;
dim = get(handles.radiobutton26,'Value');
if dim == 1
    contv1 = 2;
end

velocidadx;
velocidady;
velocidadz;

velocidadAgentes (vector2,1)= [str2double(velocidadx)];
set(handles.text30,'String',velocidadAgentes(vector2,1));
set(handles.pushbutton17,'Value',0);

velocidadAgentes (vector2,2)= [str2double(velocidady)];
set(handles.text31,'String',velocidadAgentes(vector2,2));
set(handles.pushbutton17,'Value',0);

if contv1 == 3
velocidadAgentes (vector2,3)= [str2double(velocidadz)];
set(handles.text32,'String',velocidadAgentes(vector2,3));
set(handles.pushbutton17,'Value',0);
end 
velocidadAgentes;
vector2 ;
set(handles.text29,'String',"Ingresado Vector Vi "+vector2)

vector2=vector2+1;
    
end




function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double


% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit13_Callback(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit13 as text
%        str2double(get(hObject,'String')) returns contents of edit13 as a double


% --- Executes during object creation, after setting all properties.
function edit13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit14_Callback(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit14 as text
%        str2double(get(hObject,'String')) returns contents of edit14 as a double


% --- Executes during object creation, after setting all properties.
function edit14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit15_Callback(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit15 as text
%        str2double(get(hObject,'String')) returns contents of edit15 as a double


% --- Executes during object creation, after setting all properties.
function edit15_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in salirzoom.
function salirzoom_Callback(hObject, eventdata, handles)
set(handles.salirzoom,'String','Salir zoom')
set(handles.pushbutton6,'String','Zoom personal')
set(handles.popupmenu3,'visible','off')
set(handles.text13,'visible','off')
set(handles.uibuttongroup3,'visible','off')
set(handles.radiobutton11,'Value',0)
set(handles.pushbutton6,'visible','on')
set(handles.salirzoom,'visible','off')



function edit16_Callback(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit16 as text
%        str2double(get(hObject,'String')) returns contents of edit16 as a double


% --- Executes during object creation, after setting all properties.
function edit16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --------------------------------------------------------------------
function universidad_Callback(hObject, eventdata, handles)
web https://www.utem.cl/ -browser


% --------------------------------------------------------------------
function guardar_Callback(hObject, eventdata, handles)
% hObject    handle to guardar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% exportgraphics(,'Agentes.png','Resolution',300)
% t = tiledlayout(2,1);
% exportgraphics(gcf,'vectorfig.pdf','ContentType','vector')


% --- Executes on button press in pushbutton19.
function pushbutton19_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% frontControlPlus = str2double(get(handles.slider2,'MAX'))
global columna2d
frontControlPlus = get(handles.slider2,'MAX') + 100;
limFrontControl1 = get(handles.slider2,'Value');
limBackControl1 = get(handles.slider7,'Value');



if frontControlPlus >= columna2d
set(handles.slider2,'MIN',0,'MAX',columna2d);
set(handles.slider7,'MIN',0,'MAX',columna2d);
% frontControlPlus = get(handles.slider2,'MAX');
else
    set(handles.slider2,'MIN',0,'MAX',frontControlPlus)
    set(handles.slider7,'MIN',0,'MAX',frontControlPlus)
end

if limFrontControl1 > frontControlPlus
    set(handles.slider2,'Value',frontControlPlus)        
end
if limBackControl1 > frontControlPlus
    set(handles.slider2,'Value',frontControlPlus)        
end

% BackControl = get(handles.slider2,'MAX')
% set(handles.slider2,'Value',frontControlPlus)



% --- Executes on button press in pushbutton20.
function pushbutton20_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
zoomRest = get(handles.slider10,'MIN')
zoomPlus = get(handles.slider10,'MAX') + 5
% if zoomRest == 0 
%     set(handles.slider10,'MIN')
% end
set(handles.slider10,'MIN',zoomRest,'MAX',zoomPlus);

% --- Executes on button press in pushbutton21.
function pushbutton21_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
frontControlSubtraction = get(handles.slider2,'MAX') - 100;

set(handles.slider2,'MIN',0,'MAX',frontControlSubtraction);
set(handles.slider7,'MIN',0,'MAX',frontControlSubtraction);
limFrontControl2 = get(handles.slider2,'Value');
limBackControl2 = get(handles.slider7,'Value');

if limFrontControl2 > frontControlSubtraction
    set(handles.slider2,'Value',frontControlSubtraction)
end
if limBackControl2 > frontControlSubtraction
    set(handles.slider7,'Value',frontControlSubtraction)
end



% --- Executes on button press in pushbutton22.
function pushbutton22_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
zoomRest = get(handles.slider10,'MIN') - 0.01
if zoomRest <= 0.1
    zoomRest = zoomRest - 0.01 
end
if zoomRest <= 0 
   zoomRest = 0.01
end
set(handles.slider10,'MIN',zoomRest);



% --- Executes on button press in pushbutton23.
function pushbutton23_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton24.
function pushbutton24_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton25.
function pushbutton25_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ejeyPlus = get(handles.ejey,'MAX') + 1
set(handles.ejey,'MIN',-ejeyPlus,'MAX',ejeyPlus);


% --- Executes on button press in pushbutton26.
function pushbutton26_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ejeyPlus = get(handles.ejey,'MAX') - 1
set(handles.ejey,'MIN',-ejeyPlus,'MAX',ejeyPlus);


% --- Executes on button press in pushbutton27.
function pushbutton27_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ejexPlus = get(handles.ejex,'MAX') + 1
set(handles.ejex,'MIN',-ejexPlus,'MAX',ejexPlus);


% --- Executes on button press in pushbutton28.
function pushbutton28_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ejexPlus = get(handles.ejex,'MAX') - 1
set(handles.ejex,'MIN',-ejexPlus,'MAX',ejexPlus);


% --- Executes on button press in pull.
function pull_Callback(hObject, eventdata, handles)
% hObject    handle to pull (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global refreshOn zoomRefresh frontRefresh ejeyRefresh ejexRefresh
set(handles.deseado,'Value',0);
dos=get(handles.radiobutton26,'Value');
tres=get(handles.radiobutton27,'Value');
if refreshOn == 1
zoomRefresh = num2str(get(handles.slider10,'MAX'));
frontRefresh = num2str(get(handles.slider2,'MAX'));
ejeyRefresh = num2str(get(handles.ejey,'MAX'));
ejexRefresh = num2str(get(handles.ejex,'MAX'));
refreshOn = 0;
end
cont = get(handles.pull,'String');
if cont == "Refresh"
if dos==1
    set(handles.uitable10,'Visible','on');
    A = {'Limit Front Control';'y';'x';'zoom'};
    B = {frontRefresh;ejeyRefresh;ejexRefresh;zoomRefresh };
    variables = [A B];    
    set(handles.uitable10,'data',variables);
end

if tres == 1
    set(handles.uitable10,'Visible','on');
    A = {'Limit Front Control';'y';'x';'zoom'};
    B = {frontRefresh;ejeyRefresh;ejexRefresh;zoomRefresh };
    variables = [A B];
    set(handles.uitable10,'data',variables);
end


    set(handles.pull,'String','Push')
    set(handles.uitable10,'Visible','on')
end    
if cont == "Push"
recibido=get(handles.uitable10,'data');

  r1 = recibido(1,2);
  frontRefresh = str2double(r1);
  r2 = recibido(2,2);
  ejeyRefresh = str2double(r2);
  r3 = recibido(3,2);
  ejexRefresh = str2double(r3);
  r4 = recibido(4,2);
  zoomRefresh = str2double(r4);
  zoomValue = get(handles.slider10,'Value');
  frontValue = get(handles.slider2,'Value');
  ejeyValue = get(handles.ejey,'Value');
  ejexValue = get(handles.ejex,'Value');
  if zoomValue >= zoomRefresh
    set(handles.slider10,'Value',zoomRefresh);  
  end
  if frontValue >= frontRefresh
    set(handles.slider2,'Value',frontRefresh);  
  end
  if ejeyValue >= ejeyRefresh
    set(handles.ejey,'Value',ejeyRefresh);  
  end
  if ejexValue >= ejexRefresh
    set(handles.ejex,'Value',ejexRefresh);  
  end
 
  frontRefresh;
  frontRefresh;
  zoomRefresh;
  
  set(handles.slider2,'MIN',0,'MAX',frontRefresh);
  set(handles.slider7,'MIN',0,'MAX',frontRefresh);
  set(handles.slider10,'MIN',1,'MAX',zoomRefresh);
  set(handles.ejey,'MIN',-ejeyRefresh,'MAX',ejeyRefresh);
  set(handles.ejex,'MIN',-ejexRefresh,'MAX',ejexRefresh);
  
  set(handles.uitable10,'Visible','on')
  

%   set(handles.pull,'String','Refresh')
end


% --- Executes on button press in pushbutton30.
function pushbutton30_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton30 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global grabar archivo
grabar = 1;
archivo = 0;

% --- Executes on button press in pushbutton31.
function pushbutton31_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global archivo grabar
archivo = 1;
grabar = 0;



function edit18_Callback(hObject, eventdata, handles)
% hObject    handle to edit18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit18 as text
%        str2double(get(hObject,'String')) returns contents of edit18 as a double


% --- Executes during object creation, after setting all properties.
function edit18_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit17_Callback(hObject, eventdata, handles)
% hObject    handle to edit17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit17 as text
%        str2double(get(hObject,'String')) returns contents of edit17 as a double


% --- Executes during object creation, after setting all properties.
function edit17_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit19_Callback(hObject, eventdata, handles)
% hObject    handle to edit19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit19 as text
%        str2double(get(hObject,'String')) returns contents of edit19 as a double


% --- Executes during object creation, after setting all properties.
function edit19_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit20_Callback(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit20 as text
%        str2double(get(hObject,'String')) returns contents of edit20 as a double


% --- Executes during object creation, after setting all properties.
function edit20_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit21_Callback(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit21 as text
%        str2double(get(hObject,'String')) returns contents of edit21 as a double


% --- Executes during object creation, after setting all properties.
function edit21_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in iniciales.
function iniciales_Callback(hObject, eventdata, handles)
iniciales = get(handles.iniciales,'Value');
dimension = get(handles.radiobutton26,'Value');
if iniciales == 1
    set(handles.text28,'String','Ingrese Vectores Posición')
    set(handles.text29,'String','Ingrese Vectores Velocidad')
    set(handles.edit12,'String','x')
    set(handles.edit13,'String','y')
    set(handles.edit14,'String','z')
    set(handles.edit19,'String','x')
    set(handles.edit20,'String','y')
    set(handles.edit21,'String','z')
    set(handles.text28,'Visible','on')
    set(handles.text25,'Visible','on')
    set(handles.text26,'Visible','on')
    set(handles.text29,'Visible','on')
    set(handles.text30,'Visible','on')
    set(handles.text31,'Visible','on')
    set(handles.pushbutton17,'Visible','on')
    set(handles.edit12,'Visible','on')
    set(handles.edit13,'Visible','on')
    set(handles.edit19,'Visible','on')
    set(handles.edit20,'Visible','on')
    
    if dimension == 0
        set(handles.edit21,'Visible','on')
        set(handles.edit14,'Visible','on')
        set(handles.text27,'Visible','on')
        set(handles.text32,'Visible','on')
    end
    
    if dimension == 1
        set(handles.edit21,'Visible','off')
        set(handles.edit14,'Visible','off')
        set(handles.text27,'Visible','off')
        set(handles.text32,'Visible','off')
        
    end
end

if iniciales == 0
    set(handles.text28,'String','Ingrese Vectores')
    set(handles.text28,'Visible','off')
    set(handles.text25,'Visible','off')
    set(handles.text26,'Visible','off')
    set(handles.text29,'Visible','off')
    set(handles.text30,'Visible','off')
    set(handles.text31,'Visible','off')
    set(handles.pushbutton17,'Visible','off')
    set(handles.edit12,'Visible','off')
    set(handles.edit13,'Visible','off')
    set(handles.edit19,'Visible','off')
    set(handles.edit20,'Visible','off')
    set(handles.edit21,'Visible','off')
        set(handles.edit14,'Visible','off')
        set(handles.text27,'Visible','off')
        set(handles.text32,'Visible','off')
end
    
