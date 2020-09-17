function varargout = Slider(varargin)
% SLIDER MATLAB code for Slider.fig
%      SLIDER, by itself, creates a new SLIDER or raises the existing
%      singleton*.
%
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
%      kkk.  All inputs are passed to Slider_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Slider

% Last Modified by GUIDE v2.5 03-Sep-2020 19:59:41

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


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% close all
% clc
%function [value,th]=consensus3(gamma0,lambda0)
global np ndim sigma K beta pp flag alpha M zdes
flagcolision=0;
tic
flag=0;
M=50;

ndim=2;
%sigma=0 singular influence
sigma=0;
K=100;
beta=0.5;
alpha=1.1;
alpha1=10;
pp=2;
np=3; %number of agents
ti=0;
tf=40;
dt=.001;
t=ti:dt:tf;
nsteps=length(t);


gamma0=14;
lambda0=42.3;

x00=zeros(np,ndim);
v00=zeros(np,ndim);

%generate random ci
for i=1:np
    for kk=1:ndim
        x00(i,kk)=2*rand-1;
        v00(i,kk)=2*rand-1;
    end
end
vbar0=1/np*sum(v00,1);
vbar2=1/np*sum(v00,1);
zdes=zeros(np-1,ndim);

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

% SQUARE
np=4;
modv=1;
side=2;

% SQUARE 2
x00(1,:)=[-side 0]; v00(1,:)=-modv*[1 0];
x00(2,:)=[0 side]; v00(2,:)=-modv*[-1. 0];
x00(3,:)=[side 0]; v00(3,:)=-modv*[0 -1];
x00(4,:)=[0 -side]; v00(4,:)=-modv*[0 1];

zdes(1,:)=[side side];
zdes(2,:)=[side -side];
zdes(3,:)=[-side -side];

x=zeros(2*np,ndim,nsteps);
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
[t,X]=ode23s(@csdynmatlab,0:dt:tf,xinitt);
X=X';
ftime=length(t);
x1=X(1:np,:);
x2=X(2*np+1:3*np,:);
v1=X(np+1:2*np,:);
v2=X(3*np+1:4*np,:);

col = jet(1);
hold on
d=1;
ip=1;
dd=1;
sss=0;
while dd==1
% if p==1
s= get(handles.slider2,'Value');
ss=round(s);
if ss==0
ss=1
end
set(handles.text2,'String',ss);
if sss~=ss
 
% y= -1*ss 
% plot(ss,y,'Marker','x')
if d~=1

if ss~=sss  
   ip=sss; 
   %ss=sss;
end
end
cla
hold on
    plot(x1(1,1:ss),x2(1,1:ss),'Marker','.','Color','black')
    plot(x1(2,1:ss),x2(2,1:ss),'Marker','.','Color','green')
    plot(x1(3,1:ss),x2(3,1:ss),'Marker','.','Color','red')
    plot(x1(4,1:ss),x2(4,1:ss),'Marker','.','Color','blue')
    pause(0.001)
hold off

% pause(0.001)
sss=ss;
d=0
end
%     plot(x1(1,ss),x2(1,ss),'Marker','x','Color','red')
%     plot(x1(2,ss),x2(2,ss),'Marker','x','Color','green')
%     plot(x1(3,ss),x2(3,ss),'Marker','x','Color','blue')
%     plot(x1(4,ss),x2(4,ss),'Marker','x','Color','black')
   
pause(0.01)


% for i=1:np
%     plot(x1(i,:),x2(i,:),'Color',col(i,:))
%     plot(x1(i,1),x2(i,1),'Color',col(i,:),'Marker','x','MarkerSize',10)
%     plot(x1(i,ftime),x2(i,ftime),'Color',col(i,:),'Marker','square','MarkerSize',10)
% end

% a=get(gca,'xlim');b=get(gca,'ylim');
%  a=a+[-2 2];b=b+[-2 2];
% title('Trajectories')
end


% --- Executes during object creation, after setting all properties.
function text2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in play.
function play_Callback(hObject, eventdata, handles)



% --- Executes on button press in pausa.
function pausa_Callback(hObject, eventdata, handles)
p=get(handles.pausa,'Value');
