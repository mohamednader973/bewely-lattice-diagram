function varargout = Hv(varargin)
% HV MATLAB code for Hv.fig
%      HV, by itself, creates a new HV or raises the existing
%      singleton*.
%
%      H = HV returns the handle to a new HV or the handle to
%      the existing singleton*.
%
%      HV('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in HV.M with the given input arguments.
%
%      HV('Property','Value',...) creates a new HV or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Hv_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Hv_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Hv

% Last Modified by GUIDE v2.5 31-May-2021 20:11:27

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Hv_OpeningFcn, ...
                   'gui_OutputFcn',  @Hv_OutputFcn, ...
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


% --- Executes just before Hv is made visible.
function Hv_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Hv (see VARARGIN)

% Choose default command line output for Hv
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Hv wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Hv_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
lineNum1=get(handles.edit9,'string');
lineNum=str2num(lineNum1);
line_length1=get(handles.edit2,'string');
line_length=str2num(line_length1);
propagation_velocity1=get(handles.edit5,'string');
propagation_velocity=str2num(propagation_velocity1);
line_impedances1=get(handles.edit1,'string');
line_impedances=str2num(line_impedances1);
%clear;
%clc;
%taos=[[0 2/11];[20/11 2]];
%phis=[[-1 -9/11];[9/11 1]];
%linelengths
%wavespeed
%lineimpedances
%lineNum=2;
%times=[1.5 2];
%inputs code %
 
%propagation_velocity=[300 150];
%line_length=[450 300];
times=line_length./propagation_velocity;
%line_impedances=[400 40];
%lineNum=2;
phis=[];
taos=[];
for asd=1:1:lineNum
    if asd==1
        if (lineNum)==2
            phis(end+1,:)=[-1 (line_impedances(asd+1)-line_impedances(asd))/(line_impedances(asd+1)+line_impedances(asd))] 
            %phis(end+1,:)=[(line_impedances(asd)-line_impedances(asd+1))/(line_impedances(asd+1)+line_impedances(asd)) 1]
            %        taos    %
            taos(end+1,:)=[0 2*(line_impedances(asd+1))/(line_impedances(asd+1)+line_impedances(asd))]
            %taos(end+1,:)=[2*(line_impedances(asd))/(line_impedances(asd+1)+line_impedances(asd)) 2]
        else
            phis(end+1,:)=[-1 (line_impedances(asd+1)-line_impedances(asd))/(line_impedances(asd+1)+line_impedances(asd))]
            taos(end+1,:)=[0 2*(line_impedances(asd+1))/(line_impedances(asd+1)+line_impedances(asd))]
            
        end   
    elseif asd==lineNum
        phis(end+1,:)=[(line_impedances(asd-1)-line_impedances(asd))/(line_impedances(asd-1)+line_impedances(asd)) 1] 
        taos(end+1,:)=[2*(line_impedances(asd-1))/(line_impedances(asd-1)+line_impedances(asd)) 2]
    else
        phis(end+1,:)=[(line_impedances(asd-1)-line_impedances(asd))/(line_impedances(asd)+line_impedances(asd-1)) (line_impedances(asd+1)-line_impedances(asd))/(line_impedances(asd)+line_impedances(asd+1))]
        taos(end+1,:)=[(2*line_impedances(asd-1))/(line_impedances(asd)+line_impedances(asd-1)) (2*line_impedances(asd+1))/(line_impedances(asd)+line_impedances(asd+1))]
    end
end
%inputs code %
times_ext=times;
time1=times(1);
times(1)=[];
inputvoltage=100; %unit is kv
% for the first line section %
for j=1:1:20
    if j==1
    incident_volt(j,1)= 0; %at beginning of  line section
    incident_volt(j,2)=0;
    transmitted_volt(j,1)=inputvoltage; %at end of  line section
    transmitted_volt(j,2)=time1;
    reflected_volt(j,1)=inputvoltage*phis(1,2);  %at end of  line section
    reflected_volt(j,2)=time1;
    else
    incident_volt(j,1)= transmitted_volt(j-1,1)*phis(1,2); %at beginning of  line section
    incident_volt(j,2)=transmitted_volt(j-1,2)+time1;
    transmitted_volt(j,1)=incident_volt(j,1)*phis(1,1); %at end of  line section
    transmitted_volt(j,2)=incident_volt(j,2)+time1;
    reflected_volt(j,1)=transmitted_volt(j,1)*phis(1,2);  %at beginning of  line section
    reflected_volt(j,2)=transmitted_volt(j,2);
    end   
end
% for the first line section %
 
 
%forward effect %
for i=1:2:(2*lineNum)-2
     H=ceil(i/2);
     transvoltagephis(:,1)=transmitted_volt(:,i).*taos(H,2);
     for j =1:1:length(transmitted_volt)
        currentTimeMatrix(j,1)=[times(H);];
     end
     transvoltagephis(:,2)=transmitted_volt(:,i+1)+currentTimeMatrix;
     transmitted_volt=[transmitted_volt,transvoltagephis];
end
for k=1:1:length(times)
    %2k+2
    reflected_volt(:,2*k+1)=transmitted_volt(:,2*k+1);
    incident_volt(:,2*k+1)=reflected_volt(:,2*k+1);
    reflected_volt(:,2*k+2)=transmitted_volt(:,2*k)+times(k);
    incident_volt(:,2*k+2)=reflected_volt(:,2*k+2)+times(k);
end
%forward effect %
 
 
%backward effect %
t=size(incident_volt);
columns=t(2)-2;
columns1=t(2);
columns2=t(2)-1;
for o=3:2:columns1
    Y=ceil(o/2);
    flag=3;
%     if o==flag   % 3 1 2
%         col=1;
%     elseif mod(Y,2)~=0 %5 3 4
%         col=Y;
%     elseif mod(Y,2)==0 %7 5 6      %9 y=5 -->7,8       %11 9,10    %13 11 12    %15   13 14
%         col=Y+1;
%     end
    col=o-2;
    incident_volt_backward(:,col)=incident_volt(:,o).*taos(Y,1); 
    incident_volt_backward(:,col+1)=incident_volt(:,o+1);
    incident_volt_zwischen(:,col)=incident_volt(:,o).*phis(Y,1);
    incident_volt_zwischen(:,col+1)=incident_volt(:,o+1);
    incident_volt_forkward(:,col)=incident_volt(:,o).*taos(Y,1);
    incident_volt_forkward(:,col+1)=incident_volt(:,o+1)+times_ext(Y-1);
    transmitted_volt_forkward(:,col)=incident_volt_forkward(:,col).*phis(Y-1,1);
    transmitted_volt_forkward(:,col+1)= incident_volt_forkward(:,col+1)+times_ext(Y-1); 
    transmitted_volt_backward(:,col)=incident_volt(:,o).*phis(Y,1);
    transmitted_volt_backward_ext(:,col)=transmitted_volt_forkward(:,col).*taos(Y-1,2);
    transmitted_volt_backward_ext(:,col+1)=transmitted_volt_forkward(:,col+1)+times_ext(Y-1);
    %transmitted_volt_forward(:,col)=transmitted_volt(:,col).*taos(col-1,2)
end
for e=4:2:columns1
            transmitted_zwischen(:,e-3)=reflected_volt(:,e-1);
            transmitted_zwischen(:,e-2)=reflected_volt(:,e)-times_ext(e/2);
            transmitted_zwischen_ext(:,e-3)=transmitted_volt_forkward(:,(e/2)-1).*taos((e/2)-1,2);
            transmitted_zwischen_ext(:,e-2)=transmitted_volt_forkward(:,(e/2));
            transmitted_zwischen_ext2(:,e-3)=transmitted_zwischen_ext(:,(e/2)-1);
            transmitted_zwischen_ext2(:,e-2)=transmitted_zwischen_ext(:,e/2)+times_ext(e/2);
%     if mod(e,4)==0
%         if e==4   %4  1 2         %6  3 4      %8  5 6             %10 7 8     %12 9 10
%             transmitted_zwischen(:,(e/2)-1)=reflected_volt(:,e-1);
%             transmitted_zwischen(:,e/2)=reflected_volt(:,e)-times_ext(e/2);
%             transmitted_zwischen_ext(:,(e/2)-1)=transmitted_volt_forkward(:,(e/2)-1).*taos((e/2)-1,2);
%             transmitted_zwischen_ext(:,e/2)=transmitted_volt_forkward(:,(e/2));
%             transmitted_zwischen_ext2(:,(e/2)-1)=transmitted_zwischen_ext(:,(e/2)-1);
%             transmitted_zwischen_ext2(:,e/2)=transmitted_zwischen_ext(:,e/2)+times_ext(e/2);
%         else
%             transmitted_zwischen(:,(e/2)+1)=reflected_volt(:,e-1);
%             transmitted_zwischen(:,(e/2)+2)=reflected_volt(:,e)-times_ext(e/2);
%             transmitted_zwischen_ext(:,(e/2)+1)=transmitted_volt_forkward(:,(e/2)-1).*taos((e/2)-1,2);
%             transmitted_zwischen_ext(:,(e/2)+2)=transmitted_volt_forkward(:,(e/2));
%             transmitted_zwischen_ext2(:,(e/2)+1)=transmitted_zwischen_ext(:,(e/2)-1);
%             transmitted_zwischen_ext2(:,(e/2)+2)=transmitted_zwischen_ext(:,e/2)+times_ext(e/2);
%         end
    
%     elseif mod(e,4)~=0
%     transmitted_zwischen(:,(e/2))=reflected_volt(:,e-1);
%     transmitted_zwischen(:,(e/2)+1)=reflected_volt(:,e)-times_ext(e/2);
%     transmitted_zwischen_ext(:,(e/2))=transmitted_volt_forkward(:,(e/2)-1).*taos((e/2)-1,2);
%     transmitted_zwischen_ext(:,(e/2)+1)=transmitted_volt_forkward(:,(e/2));
%     transmitted_zwischen_ext2(:,(e/2))=transmitted_zwischen_ext(:,(e/2)-1);
%     transmitted_zwischen_ext2(:,(e/2)+1)=transmitted_zwischen_ext(:,e/2)+times_ext(e/2);
%     end
     
end
z=size(transmitted_volt);
z1=size(transmitted_zwischen_ext2);
 
currentTimeMatrixoverall=repmat(times,length(transmitted_volt),1);
for c=2:2:columns
    transmitted_volt_backward(:,c)=incident_volt(:,c+2)+currentTimeMatrixoverall(:,c/2);
end
tt=size(transmitted_volt_backward);
%backward effect %
% first line 
overall_transmitted_parttwo=[transmitted_volt(:,z(2)-1:end);transmitted_zwischen_ext2(:,z1(2)-1:end);transmitted_volt_backward(:,tt(2)-1:end)];  %final line
overall_transmitted_partthree=[transmitted_zwischen;incident_volt_backward;transmitted_zwischen_ext]; %middle lines 
final_recieved_voltage(:,1)=overall_transmitted_parttwo(:,1).*taos(lineNum,2);
final_recieved_voltage(:,2)=overall_transmitted_parttwo(:,2);
Q=sortrows(final_recieved_voltage,2);
arr=[];
% final line voltage %
for g=1:1:length(Q)-1
    if Q(g,2)==Q(g+1,2)
        final_recieved_voltage_Q(g,1)=Q(g,1)+Q(g+1,1);
        final_recieved_voltage_Q(g,2)=Q(g,2);
        arr(end+1)=g+1;
           
    else
        final_recieved_voltage_Q(g,1)=Q(g,1);
        final_recieved_voltage_Q(g,2)=Q(g,2);
    end   
end
final_recieved_voltage_Q(arr,:)=[];
 
% final line voltage %
 
%intermediate lines voltages %
w=size(overall_transmitted_partthree);
QW=w(2);
arr2=[];
 
for columns=2:2:QW
    var=sortrows(overall_transmitted_partthree,columns)
    matrix(:,columns-1:columns)=var(:,columns-1:columns);
end
v=[];
iterations=20;
matrixsize=size(matrix)
bu=[];
for po=1:2:matrixsize(2)
    v=transpose(matrix(:,po:po+1));
    v=[bu v]
    bu=v;
       
end
 
length(v);
 
arr2=[];
for s=1:1:(lineNum)-1
    for a=(3*iterations*(s-1))+1:1:(3*iterations*s)
        v(3,a)=s;
        %V_Q(3,a)=v(3,a)
    end
end
for h=1:1:length(v)-1
    if v(2,h)==v(2,h+1)
        V_Q(1,h)=v(1,h)+v(1,h+1);
        V_Q(2,h)=v(2,h);
        arr2(end+1)=h+1;
        V_Q(3,h)=v(3,h);
    else
        V_Q(1,h)=v(1,h);
        V_Q(2,h)=v(2,h);
        V_Q(3,h)=v(3,h);
    end
    if h==length(v)-1
        V_Q(1,h+1)=v(1,h+1);
        V_Q(2,h+1)=v(2,h+1);
        V_Q(3,h+1)=v(3,h+1);
    end
end
V_Q(:,arr2)=[];
%intermediate lines voltages %
 
%commulative voltages & plotting %
sum=0;
for x=1:1:length(final_recieved_voltage_Q)
    final_recieved_voltage_commulative(x,1)=sum+final_recieved_voltage_Q(x,1);
    sum=final_recieved_voltage_commulative(x,1);
    final_recieved_voltage_commulative(x,2)=final_recieved_voltage_Q(x,2);
end
figure;
stairs(final_recieved_voltage_commulative(:,2),final_recieved_voltage_commulative(:,1),'r');
legend('end section plot');
total=0;
final_recieved_current_commulative=final_recieved_voltage_commulative./line_impedances(end);
figure;
stairs(final_recieved_current_commulative(:,2),final_recieved_current_commulative(:,1),'g');
legend('end section current plot');
for x=1:1:length(V_Q)
    V_Q_commulative(1,x)=total+V_Q(1,x);
    total=V_Q_commulative(1,x);
    V_Q_commulative(2,x)=V_Q(2,x);
    V_Q_commulative(3,x)=V_Q(3,x);
end
plotsNum=V_Q_commulative(3,end);
arr3=[];
for z=2:1:length(V_Q_commulative)
    if V_Q_commulative(3,z) ~=V_Q_commulative(3,z-1)
        arr3(end+1)=z;
    end  
end
if isempty(arr3)
    figure;
    stairs(V_Q_commulative(2,:),V_Q_commulative(1,:),'b');
    legend('section 1 plot');
else
    for c=1:1:plotsNum
        if c==1
        figure;
        hold on;
        stairs(V_Q_commulative(2,1:arr3(c)-1),V_Q_commulative(1,1:arr3(c)-1));
        legendInfo = sprintf('section %d plot', c);
        legend(legendInfo);
        elseif c==plotsNum
        figure;
        hold on;
        stairs(V_Q_commulative(2,arr3(end):end),V_Q_commulative(1,arr3(end):end));
        legendInfo2 = sprintf('section %d plot', c);
        legend(legendInfo2);   
        elseif c>1
        figure;
        hold on;
        stairs(V_Q_commulative(2,arr3(c-1):arr3(c)-1),V_Q_commulative(1,arr3(c-1):arr3(c)-1));
        legendInfo1 = sprintf('section %d plot', c);
        legend(legendInfo1);
        
        end
    end
end
%commulative voltages & plotting %
 y1=num2str(final_recieved_voltage_commulative)
 y2=num2str(final_recieved_current_commulative)
 set(handles.edit7,'string',y1);
 set(handles.edit8,'string',y2);
 axes(handles.axes1);
 stairs(final_recieved_voltage_commulative(:,2),final_recieved_voltage_commulative(:,1),'r');
 legend('end section voltage plot');
 axes(handles.axes2);
 stairs(final_recieved_voltage_commulative(:,2),final_recieved_current_commulative(:,1),'g');
 legend('end section current plot');
 
 axes(handles.axes4);
 plot(1:100,inputvoltage,'g');
 legend('input voltage plot');
 if isempty(arr3)
    axes(handles.axes5);
    stairs(V_Q_commulative(2,:),V_Q_commulative(1,:),'b');
    legend('section 1 plot');
else
    for c=1:1:plotsNum
        if c==1
        axes(handles.axes5);
        stairs(V_Q_commulative(2,1:arr3(c)-1),V_Q_commulative(1,1:arr3(c)-1));
        legendInfo = sprintf('section %d plot', c);
        legend(legendInfo);
        hold on;
        elseif c==plotsNum
        axes(handles.axes5);
        stairs(V_Q_commulative(2,arr3(end):end),V_Q_commulative(1,arr3(end):end));
        legendInfo2 = sprintf('section %d plot', c);
        legend(legendInfo2);   
        hold on; 
        elseif c>1
        axes(handles.axes5);
        stairs(V_Q_commulative(2,arr3(c-1):arr3(c)-1),V_Q_commulative(1,arr3(c-1):arr3(c)-1));
        legendInfo1 = sprintf('section %d plot', c);
        legend(legendInfo1);
        hold on; 
        end
    end
end
 




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
