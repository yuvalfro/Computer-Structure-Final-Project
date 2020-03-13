function varargout = gui1(varargin)
% GUI1 MATLAB code for gui1.fig
%      GUI1, by itself, creates a new GUI1 or raises the existing
%      singleton*.
%
%      H = GUI1 returns the handle to a new GUI1 or the handle to
%      the existing singleton*.
%
%      GUI1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI1.M with the given input arguments.
%
%      GUI1('Property','Value',...) creates a new GUI1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui1

% Last Modified by GUIDE v2.5 11-Aug-2019 18:53:52

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui1_OpeningFcn, ...
                   'gui_OutputFcn',  @gui1_OutputFcn, ...
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


% --- Executes just before gui1 is made visible.
function gui1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui1 (see VARARGIN)

% Choose default command line output for gui1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% This sets up the initial plot - only do when we are invisible
% so window can get raised using gui1.
if strcmp(get(hObject,'Visible'),'off')
    axes(handles.axes1);
    grid on;
    handles.axes1.CameraPosition = [-95 -95 105];
    handles.axes1.YColor = 'none';
    handles.axes1.XColor = 'none';
    handles.axes1.ZColor = 'none';   
end

% UIWAIT makes gui1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = gui1_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla reset;
axes(handles.axes1);
grid on;
if ~isempty(instrfind)
     fclose(instrfind);
     delete(instrfind);
end

s = serial('COM10','baudrate',9600,'Terminator','CR','StopBits',1);
popup_sel_index = get(handles.popupmenu1, 'Value');
switch popup_sel_index
    case 1
        numofrowstoscan = get(handles.popupmenu2, 'Value')+2;
        numofrowst = sprintf('%.0f',numofrowstoscan);
        st = {'3Dscan', numofrowst, '@'};
        str = strjoin(st,'');
        distanceData=zeros(5,200);
        minRowSize=200;
        EOSflag=0;
        firstInRow=0;
        rownum = 0;
        fopen(s);
        fprintf(s,str);
        while((EOSflag == 0)&&(popup_sel_index==1))
            msg = fscanf(s,'%s');
            if(strcmp(msg,''))
                pause(0.5);
            end 
            popup_sel_index = get(handles.popupmenu1, 'Value');
            if(strcmp(msg,'EndOfScan'))
                EOSflag=1;  
            else
                if(strcmp(msg,'StartRowScan'))
                   msg = fscanf(s,'%s');
%                    while(strcmp(msg,''))
%                    msg = fscanf(s,'%s');    
%                    end
                   while(strcmp(msg,'EndOfRow')~=1)
                       newdistance = str2double(msg);
                       if(firstInRow == 0)
%                            if (rownum == 0)
                              distanceRow = newdistance;
%                            else
%                             distance2 = [distance2; newdistance]; 
%                            end
                           firstInRow=1;
                       else
                          distanceRow =[distanceRow newdistance];
                       end
                       msg = fscanf(s,'%s');
                   end
                   if(minRowSize > numel(distanceRow))
                       minRowSize=numel(distanceRow);
                   end
                   rownum = rownum+1;
                   distanceRow(200)= 0;
                   distanceData(rownum,:)=distanceRow;
                   firstInRow=0;
                   
                end
            end    
        end
        if(EOSflag == 1)
        distanceData=distanceData(1:numofrowstoscan,1:minRowSize);
        [numofrows,numofcols] = size(distanceData);
        distanceData = -distanceData;
        avg = mean(distanceData,2); 
        minavg = max(avg);
        if (distanceData(1,1) == 0)
            distanceData(1,1) = avg(1);
        end
        for k=1:numofrows
            for i=1:numofcols
                if (distanceData(k,i) <= 2*avg(k))
                    distanceData(k,i) = minavg;
                end
            end
        end
     
        figure(1);    %Shape without changes
%         surf(distanceData);
        distanceDataMatrix = distanceData;
%         distanceData = reshape(distanceData.',1,[]);
%         distanceData = transpose(distanceData);
        distanceData = reshape(distanceData,numel(distanceData),1);
        m = 30/numofcols;
        n = 30/numofrows;
        x=1:m:(1+m*(numofcols-1));
        y=1:n:30;
        [xd,yd]=meshgrid(x,y);
        surf(xd,yd,distanceDataMatrix);
        title('Shape without changes')
%       x2 = repmat(x,1,numofrows);
%       y2 = repmat(y,1,numofcols);
%       x2 = transpose(x2);
%       y2 = transpose(y2);
         x2=reshape(xd,numel(xd),1);
         y2=reshape(yd,numel(yd),1);
         F = scatteredInterpolant(x2,y2,distanceData, 'natural');
% %        F.Method = 'natural';
% %       [xd,yd]=meshgrid(x,y);
%         vq1 = F(xd,yd);
%         figure(2)      %Shape with interpulation
%         %hold on
%         surf(xd,yd,vq1);
        
        
        m = 30/(sqrt(2)*numofcols);
        n = 30/(sqrt(2)*numofrows);
        
        x3 = 1:m:(1+m*(numofcols-1));
        y3 = 1:n:(1+n*(numofrows-1));
        [xd3,yd3]=meshgrid(x3,y3);
        vq3 = F(xd3,yd3);
        figure(3)
       % hold on
        surf(xd3,yd3,vq3);
        title('Shape with interpulation')
        end
        
%         m = 30/minRowSize;
%         x=1:m:(1+m*(numofcols-1));
%         y=1:5:25;
%         [xd,yd]=meshgrid(x,y);
%         figure(2);
%         surf(xd,yd,distanceData);
        
        
    case 2
        x=6; y=4; z=2;
        xlim([-10,10]); ylim([-10,10]); zlim([-10,10]);
        f(1)=patch([-x/2,x/2,x/2,-x/2],[-y/2,-y/2,y/2,y/2],[-z/2,-z/2,-z/2,-z/2],[0 0.4470 0.7410]);
        f(2)=patch([-x/2,x/2,x/2,-x/2],[-y/2,-y/2,y/2,y/2],[z/2,z/2,z/2,z/2],[0.6350 0.0780 0.1840]	);
        f(3)=patch([-x/2,-x/2,x/2,x/2],[-y/2,-y/2,-y/2,-y/2],[-z/2,z/2,z/2,-z/2],[0.4660 0.6740 0.1880]);
        f(4)=patch([x/2,x/2,x/2,x/2],[-y/2,-y/2,y/2,y/2],[-z/2,z/2,z/2,-z/2],[0.9290 0.6940 0.1250]);
        f(5)=patch(-[x/2,x/2,x/2,x/2],[y/2,y/2,-y/2,-y/2],[-z/2,z/2,z/2,-z/2],'white');
        f(6)=patch([-x/2,-x/2,x/2,x/2],[y/2,y/2,y/2,y/2],[-z/2,z/2,z/2,-z/2],'black');      
        rotate3d on;
        %grid on;
        handles.axes1.YColor = 'none';
        handles.axes1.XColor = 'none';
        handles.axes1.ZColor = 'none';
        handles.text2.String = 'The white face is the UltraSonic side';
        handles.text2.Visible ='on';   
        handles.axes1.CameraPosition = [-95 -95 105];
        
        fclose(s);
        fopen(s);
        fprintf(s,'Sysmove@');
 %       pause(0.5);
        while (popup_sel_index == 2)
        fprintf(s,'x@');
        accx = str2double(fscanf(s,'%s'));
        
        fprintf(s,'y@');
        accy = str2double(fscanf(s,'%s'));
        
        fprintf(s,'z@');
        accz = str2double(fscanf(s,'%s'));
        
       % roll = atan(accy / sqrt(accx.^ 2 + accz.^ 2)) * 180 / pi;
       % pitch = atan(-1 * accx / sqrt(accy.^ 2 + accz.^ 2)) * 180 / pi;
       % atan is between -pi\2 to pi\2
        
        roll = atan2(accy , accz) * 180 / pi;
        pitch = atan2(-1 * accx , sqrt(accy.^ 2 + accz.^ 2)) * 180 / pi;
        %atan2 is between -pi to pi, so in one axis we flip
  
        direction = [1 0 0];
        rotate(f,direction,roll);
         
        direction = [0 1 0];
        rotate(f,direction,pitch);
        drawnow
        
        direction = [0 1 0];
        rotate(f,direction,-pitch);
       
        direction = [1 0 0];
        rotate(f,direction,-roll)
         
        end 
        
    case 3
        fopen(s);
        fprintf(s,'Telemeter@');
        k=0;
        while (popup_sel_index == 3)
            if(k~=24)
            distance = str2double(fscanf(s,'%s'));
            distanceStr = sprintf('%.1f',distance);
            str = get(handles.text1,'String');
            str1 = char(str,distanceStr);
            set(handles.text1,'String',str1);
            k=k+1;
            drawnow
            else
            k=0;
            set(handles.text1,'String','Telemeter: Distance in cm');
            drawnow
            end
        end
end


% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
     set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'3D depth scanner', 'System movement', 'Display distance'});


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbutton1.
function pushbutton1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in stop_button.
function stop_button_Callback(hObject, eventdata, handles)
% hObject    handle to stop_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in stop_button.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to stop_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2


% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
set(hObject, 'String', {'3', '4', '5','6','7','8','9','10'});
