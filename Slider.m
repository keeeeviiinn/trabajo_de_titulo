function varargout = Slider(varargin)
% SLIDER MATLAB code for Slider.fig
%      SLIDER, by itself, creates a new SLIDER or raises the existing
%      singleton*.
% holaaa mundo
%      H = SLIDER returns the handle to a new SLIDER or the handle to
%      the existing singleton*.
%
%      SLIDER('CALLBACK',hObject,eventData,handles,...) calls the local
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

% Last Modified by GUIDE v2.5 13-Oct-2020 16:51:31

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
    %%Codigo para ingresar cantidad de agentes 
npp    = get(handles.edit1,'String');%Se guarda el vaor que se ingrese en el cuadro de texto en guide en la variable "npp"
nppp   = str2double(npp); %el valor de npp se convierte en un numero
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
set(handles.pushbutton11,'String',"Vista general")
movimiento = 0;%Movimiento automatico de los agentes mediante boton

% datos = get(handles,uitable3,'Data');
% datos(:,:) = [];
% set(handles.uitable3,'Data',datos)

global np ndim sigma K beta pp flag alpha M zdes
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
alpha=1.1

pp=2; %potencia a la que esta elevada
% np=5; %number of agents


end
if deseado == 1
  recibido=get(handles.uitable10,'data');
  n=size(recibido,1);
  archivo = fopen ('datos.dat','w');
  
  for i = 1:n
      fprintf(archivo, '%s = %s \n',recibido{i,:}); % %s es formato string
  end
  fclose(archivo)
  
  v1 = recibido(1,2);
  alpha = str2double(v1)
  v2 = recibido(2,2);
  beta = str2double(v2)
  v3 = recibido(3,2);
  sigma = str2double(v3)
  v4 = recibido(4,2);
  K = str2double(v4)
  v5 = recibido(5,2);
  pp = str2double(v5)
  v6 = recibido(6,2);
  flag = str2double(v6)
  v7 = recibido(7,2);
  M = str2double(v7)
  
  
end

ndim=2;
np=nppp;
alpha1=10;
ti=0;
tf=20;
dt=.001; 
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

% bird-like flocking 
vbardir=vbar0/norm(vbar0);
anglevbdir=atan2(vbardir(2),vbardir(1));

for i=1:floor(np/2)
    zdes(i,:)=-[2*cos(anglevbdir-10*2*pi/180) 2*sin(anglevbdir-10*2*pi/180)];
end

for i=floor(np/2)+1:np-1
    zdes(i,:)=[2*cos(anglevbdir+10*2*pi/180) 2*sin(anglevbdir+10*2*pi/180)];
end

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
    centrar = get(handles.pushbutton10,'String');
    movimiento = get(handles.pushbutton5,'String');
    general = get(handles.pushbutton11,'String');
    
if movim == 20001
    movimiento = "no";
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

if  sss~=ss | sss2~=ss2 | zz ~= get(handles.slider10,'Value') | ejexx ~= get(handles.ejex,'Value') | ejeyy ~= get(handles.ejey,'Value') | centrar == "Centrado" %| general == "Ok!" 
   if  d~=1
      if  ss~=sss  
          ip=sss; 
    
      end
        
if      ss2~=sss2 
        ip=sss2;
    
end
    end
        cla%borra lo que se genera anteriormente de los plot, y asi no se sobre escriben y se optimiza
        hold on
        if centrar == "Centrado" % Realiza el centrado de los slider ejex y ejey
            set(handles.ejex,'Value',0)
            set(handles.ejey,'Value',0)
            set(handles.slider10,'Value',50)
            set(handles.pushbutton10,'String',"Centrar")
        end
        
        
            
        
for     i=1:np
        
          ejey = get(handles.ejey,'Value');
%           ejey = ejey);
          ejex = get(handles.ejex,'Value');
%           ejex = ejex;
          
          

        plot(x1(i,ss2:1:ss),x2(i,ss2:1:ss),'.','MarkerSize',8)
        plot(x1(i,ss),x2(i,ss),'o','MarkerSize',8)
        plot(x1(i,ss2),x2(i,ss2),'o','MarkerSize',8)
        
        z = get(handles.slider10,'Value');
%         z=z;
        

           
        zoom_agente = get(handles.pushbutton6,'String');
        
        
        o = get(handles.popupmenu3,'Value');% recibe el valor del menu de agente
        front = get(handles.radiobutton11,'Value');
        back = get(handles.radiobutton12,'Value');
        
        if zoom_agente == "Zoom Personal" & front == 1%% controla la cabeza del agente para realizar zoom de seguimiento
            
            xlim([x1(o,ss)-z+ejex x1(o,ss)+z+ejex])
            ylim([x2(o,ss)-z+ejey x2(o,ss)+z+ejey])%realiza el zoom de acercamiento al zoom en el menu desplegable
            
        elseif zoom_agente == "Zoom Personal" & back == 1
            xlim([x1(o,ss2)-z+ejex x1(o,ss2)+z+ejex])
            ylim([x2(o,ss2)-z+ejey x2(o,ss2)+z+ejey])
         
          %% Inicio de boton para un zoom general
%         elseif general == "Ok!"
          
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
set(handles.pushbutton11,'String',"Stop")
clc
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
set(handles.pushbutton5,'String',"Ok!")
%
function radiobutton10_Callback(hObject, eventdata, handles)
%
function pushbutton6_Callback(hObject, eventdata, handles)
set(handles.pushbutton6,'String',"Zoom Personal")
set(handles.popupmenu3,'visible','on')
set(handles.text13,'visible','on')
set(handles.uibuttongroup3,'visible','on')
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


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global np ndim sigma K beta pp flag alpha M zdes
f = uifigure('Position', [100 100 300 250],'Color','White','Name','Obtención de datos');
    t = uitable(f,'Data',[np;ndim;alpha;beta;flag;M;sigma;K;pp]);
    t.FontSize = 10;%alpha beta flag M sigma K pp   ;'Alpha' ;'Beta' ; 'Flag';'M' ;'Sigma' ;'K' ;'PP'
    t.ColumnName = {'Datos'};
    t.RowName = {'Agentes' 'Dimensión' 'Alpha' 'Beta' 'Flag' 'M' 'Sigma' 'K' 'PP'};
    t.Position = [50 50 195 190];


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
global np ndim sigma K beta pp flag alpha M zdes
set(handles.uitable10,'Visible','on');
A = {'Alpha';'Beta';'Sigma';'K';'PP';'Flag';'M'};
B = {'';'';'';'';'';'';''};
variables = [A B];
set(handles.uitable10,'data',variables);



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
