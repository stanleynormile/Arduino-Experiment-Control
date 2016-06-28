% Mass Flow Contorller Control System
% Tufts Sustainable Electromechanical Energy Lab
% Written by Stanley Normile
% User interface code generated using Matlab GUIDE

% Initialization Function - Auto Generated
function varargout = Control_ui(varargin)
% CONTROL_UI MATLAB code for Control_ui.fig
%      CONTROL_UI, by itself, creates a new CONTROL_UI or raises the existing
%      singleton*.
%
%      H = CONTROL_UI returns the handle to a new CONTROL_UI or the handle to
%      the existing singleton*.
%
%      CONTROL_UI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CONTROL_UI.M with the given input arguments.
%
%      CONTROL_UI('Property','Value',...) creates a new CONTROL_UI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Control_ui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Control_ui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Control_ui

% Last Modified by GUIDE v2.5 28-Jun-2016 18:12:06

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Control_ui_OpeningFcn, ...
                   'gui_OutputFcn',  @Control_ui_OutputFcn, ...
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


% --- Executes just before Control_ui is made visible.
function Control_ui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Control_ui (see VARARGIN)

% Choose default command line output for Control_ui
handles.output = hObject;

% Open File for output
%%%%%%%%%%%%% Edit here to change file output name %%%%%%%%%%%%%%%%%%%%%%%
t = datetime('now','TimeZone','local','Format','dMMMy_HH_mm');
S = char(t);
handles.filename = ['Flow_Data_',S,'.xlsx'];
%%%%%%%%%%%%%%%%%%%%%%%% End file output name edit %%%%%%%%%%%%%%%%%%%%%%
header = {'Time','H2 Set','H2 Measured','Air Set','Air Measured'};
xlswrite(handles.filename,header);
handles.nextCell = 2;

% Update handles structure
guidata(hObject, handles);

%% Control System Functions
% Written by Stanley Normile

function ArudinoSetup(hObject, handles)
%ARDUINOSETUP Initializes connection with Arduino and configures pins

% Find Board and Com Port
% Determine if user is manually specifying Arduino settings
% findobj function finds the specified object in the user interface and
% returns a struct that contains all the information on that UI object
auto = findobj('Tag','AutoDetect');
if auto.Value == 1
    % Find board automatically
    handles.a = arduino();
else
    % Find board manualy
    board = findobj('Tag','board');
    com = findobj('Tag','com');
    board = board.String{board.Value};
    com = com.String;
    handles.a = arduino(com, board);
end

% Configure Pins
% While it is not always mandatory to specify pin modes, it is still good
% practice to do so - the pin assignments may be changed in the UI. To
% change the default pin settings, open Control_ui.fig in GUIDE by typing
% guide into the Command Window. Double click on the edit text box of the
% desired pin to open the inspector and change the default value by
% changing the field String
% configurePin is a Matlab built in function see help configurePin
Hin = findobj('Tag', 'H_in_pin');
handles.HinPin = Hin.String;
configurePin(handles.a,handles.HinPin);

Ain = findobj('Tag', 'A_in_pin');
handles.AinPin = Ain.String;
configurePin(handles.a,handles.AinPin);

Hout = findobj('Tag', 'H_out_pin');
handles.HoutPin = strcat('D', Hout.String);
configurePin(handles.a,handles.HoutPin);

Aout = findobj('Tag', 'A_out_pin');
handles.AoutPin = strcat('D', Aout.String);
configurePin(handles.a,handles.AoutPin);

% Make Program Runnable
% Once the Arduino has been paired, alert the user and enable the run
% button
run = findobj('Tag', 'run');
run.Enable = 'on';
h = msgbox('Arduino Ready');
% Update guidata
guidata(hObject, handles);

function ArduinoControl(hObject, handles)
%ARDUINOCONTROL executes similar to Void Loop in Arduino code
% After exectuing some initial setup, loops continuously to relay
% information between Arduino and UI

a = handles.a; % Arduino object
% Initialize variables
stoploop = false;

% Find user specified loop time
readT = findobj('Tag','t_ave');
t_ave = readT.Value;

% Loops continuously while program is running
while ~stoploop
    
    % Find constant offset
    os = findobj('Tag','offset');
    offset = str2double(os.String);
    
    % Read desired mass flow rate
    h = findobj('Tag','H_setpt');
    air = findobj('Tag','A_setpt');
    handles.H_set = str2double(h.String);
    handles.A_set = str2double(air.String);
    
    % Read current flow - reads and averages for the specified time
    [handles.H_now, handles.A_now] = averageFlow(handles, t_ave);
    
    % Apply offset to flow setpoint
    handles.H_set = handles.H_set + offset;
    handles.A_set = handles.A_set + offset;
    
    % Update handles
    guidata(hObject, handles);
    
    % Write current flow to UI
    set(handles.H_flow_now, 'String', num2str(round(handles.H_now)));
    set(handles.A_flow_now, 'String', num2str(round(handles.A_now)));
    
    % Write Data to file
    % handles.nextCell tell the program where in the xls file to begin writing
    handles.nextCell = writeData(handles.filename, handles.H_set, handles.A_set, handles.H_now, handles.A_now, handles.nextCell);
    
    % Write new flow to Arudino
    % writePWMVoltage is a Matlab built in function. For more information
    % see help writePWMVoltage
    writePWMVoltage(a, handles.HoutPin, flow2v(handles.H_set));
    writePWMVoltage(a, handles.AoutPin, flow2v(handles.A_set));

    % Check for stop condition
    s = findobj('Tag','stop');
    c = findobj('Tag','close');
    if s.Value ~= 0 || c.Value ~= 0
        stoploop = true;
        % Write zero flow
        writePWMVoltage(a, handles.HoutPin, flow2v(0));
        writePWMVoltage(a, handles.AoutPin, flow2v(0));
        % Write setpoint to UI
        
        
    end
    pause(0.1);
end

% Update guidata
guidata(hObject, handles);

function [flowh, flowa] = averageFlow(handles, t_ave)
%AVERAGEFLOW Collects the flow data for a specified period of time and
%averages it. Inputs: handles, t_ave (time over which to read and average
%data)

% Begin clock
tic
i = 1;
% Run until clock reaches t_ave
% readVoltage is a Matlab built in function for more information see help
% readVoltage
clearvars vh va
while toc < t_ave
    vh(i) = readVoltage(handles.a, handles.HinPin);
    va(i) = readVoltage(handles.a, handles.AinPin);
    i = i+1;
end

% Filter Noise
vhf = medfilt1(vh);
vaf = medfilt1(va);

% Average array
vhm = mean(vhf);
vam = mean(vaf);

% Convert voltage to flow
flowh = v2flow(vhm);
flowa = v2flow(vam);

function nextCell = writeData(filename, hset, aset, hnow, anow, cell)
%WRITEDATA Writes the current and desired flow data to the output file
%along with a time stamp
% Inputs: filename, H2 setpoint, Air setpoint, current H2 flow, current Air
% flow, next open cell in the spreadsheet
% Outputs: new next open cell in the spreadsheet

% Timestamp
t = char(datetime('now','TimeZone','local','Format','HH:mm:ss'));
% Format data
data = {t,hset,hnow,aset,anow};
% Format cell call (ex. A4)
thisCell = strcat('A',num2str(cell));
% Write to excel file
xlswrite(filename,data,1,thisCell);
nextCell = cell + 1;

function flow = v2flow(v)
% V2FLOW converts voltage to flow rate
flow = v*(1000/5);

function v = flow2v(flow)
% FLOW2V converts flow rate to voltage
v = flow/(1000/5);


%% Callbacks
% Auto generated by MATLAB
% These functions call the control system functions when buttons or sliders
% in the UI are activated
% --- Outputs from this function are returned to the command line.
function varargout = Control_ui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function H_set_Callback(hObject, eventdata, handles)
% hObject    handle to H_set (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.H_set = get(hObject,'Value');
guidata(hObject, handles);
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function H_set_CreateFcn(hObject, eventdata, handles)
% hObject    handle to H_set (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function H_flow_now_Callback(hObject, eventdata, handles)
% hObject    handle to H_flow_now (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --- Executes during object creation, after setting all properties.
function H_flow_now_CreateFcn(hObject, eventdata, handles)
% hObject    handle to H_flow_now (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in run.
function run_Callback(hObject, eventdata, handles)
% hObject    handle to run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
stop = findobj('Tag', 'stop');
stop.Enable = 'on';
ArduinoControl(hObject, handles)


% --- Executes on button press in stop.
function stop_Callback(hObject, eventdata, handles)
% hObject    handle to stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on slider movement.
function A_set_Callback(hObject, eventdata, handles)
% hObject    handle to A_set (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.H_set = get(hObject,'Value');
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function A_set_CreateFcn(hObject, eventdata, handles)
% hObject    handle to A_set (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function A_flow_now_Callback(hObject, eventdata, handles)
% hObject    handle to A_flow_now (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of A_flow_now as text
%        str2double(get(hObject,'String')) returns contents of A_flow_now as a double


% --- Executes during object creation, after setting all properties.
function A_flow_now_CreateFcn(hObject, eventdata, handles)
% hObject    handle to A_flow_now (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on selection change in board.
function board_Callback(hObject, eventdata, handles)
% hObject    handle to board (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns board contents as cell array
%        contents{get(hObject,'Value')} returns selected item from board


% --- Executes during object creation, after setting all properties.
function board_CreateFcn(hObject, eventdata, handles)
% hObject    handle to board (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function com_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function com_CreateFcn(hObject, eventdata, handles)
% hObject    handle to com (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function H_in_pin_Callback(hObject, eventdata, handles)
% hObject    handle to H_in_pin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of H_in_pin as text
%        str2double(get(hObject,'String')) returns contents of H_in_pin as a double


% --- Executes during object creation, after setting all properties.
function H_in_pin_CreateFcn(hObject, eventdata, handles)
% hObject    handle to H_in_pin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function H_out_pin_Callback(hObject, eventdata, handles)
% hObject    handle to H_out_pin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of H_out_pin as text
%        str2double(get(hObject,'String')) returns contents of H_out_pin as a double


% --- Executes during object creation, after setting all properties.
function H_out_pin_CreateFcn(hObject, eventdata, handles)
% hObject    handle to H_out_pin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function A_in_pin_Callback(hObject, eventdata, handles)
% hObject    handle to A_in_pin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of A_in_pin as text
%        str2double(get(hObject,'String')) returns contents of A_in_pin as a double


% --- Executes during object creation, after setting all properties.
function A_in_pin_CreateFcn(hObject, eventdata, handles)
% hObject    handle to A_in_pin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function A_out_pin_Callback(hObject, eventdata, handles)
% hObject    handle to A_out_pin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of A_out_pin as text
%        str2double(get(hObject,'String')) returns contents of A_out_pin as a double


% --- Executes during object creation, after setting all properties.
function A_out_pin_CreateFcn(hObject, eventdata, handles)
% hObject    handle to A_out_pin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in connect.
function connect_Callback(hObject, eventdata, handles)
ArudinoSetup(hObject, handles)


% --- Executes on button press in AutoDetect.
function AutoDetect_Callback(hObject, eventdata, handles)
% hObject    handle to AutoDetect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

ob{1} = findobj('Tag','board_str');
ob{2} = findobj('Tag','board');
ob{3} = findobj('Tag','com_str');
ob{4} = findobj('Tag','com');

if get(hObject,'Value') == 0
    for i = 1:8
        ob{i}.Visible = 'on';
    end
else
    for i = 1:8
        ob{i}.Visible = 'off';
    end
end


function Air_des_Callback(hObject, eventdata, handles)
% hObject    handle to Air_des (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Air_des as text
%        str2double(get(hObject,'String')) returns contents of Air_des as a double


% --- Executes during object creation, after setting all properties.
function Air_des_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Air_des (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function A_setpt_Callback(hObject, eventdata, handles)
% hObject    handle to A_setpt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of A_setpt as text
%        str2double(get(hObject,'String')) returns contents of A_setpt as a double


% --- Executes during object creation, after setting all properties.
function A_setpt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to A_setpt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function H_setpt_Callback(hObject, eventdata, handles)
% hObject    handle to H_setpt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of H_setpt as text
%        str2double(get(hObject,'String')) returns contents of H_setpt as a double


% --- Executes during object creation, after setting all properties.
function H_setpt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to H_setpt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in close.
function close_Callback(hObject, eventdata, handles)
% hObject    handle to close (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close(handles.figure1);


% --- Executes on button press in propCtrl.
function propCtrl_Callback(hObject, eventdata, handles)
% hObject    handle to propCtrl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

ob1 = findobj('Tag','Kp_str');
ob2 = findobj('Tag','Kp');

if get(hObject,'Value')
    ob1.Visible = 'on';
    ob2.Visible = 'on';
else
    ob1.Visible = 'off';
    ob2.Visible = 'off';
end



function Kp_Callback(hObject, eventdata, handles)
% hObject    handle to Kp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Kp as text
%        str2double(get(hObject,'String')) returns contents of Kp as a double


% --- Executes during object creation, after setting all properties.
function Kp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Kp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in def_pins.
function def_pins_Callback(hObject, eventdata, handles)
% hObject    handle to def_pins (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

ob{1} = findobj('Tag','pin_str_1');
ob{2} = findobj('Tag','pin_str_2');
ob{3} = findobj('Tag','pin_str_3');
ob{4} = findobj('Tag','pin_str_4');
ob{5} = findobj('Tag','H_in_pin');
ob{6} = findobj('Tag','H_out_pin');
ob{7} = findobj('Tag','A_in_pin');
ob{8} = findobj('Tag','A_out_pin');

if get(hObject,'Value') == 0
    for i = 1:8
        ob{i}.Visible = 'on';
    end
else
    for i = 1:8
        ob{i}.Visible = 'off';
    end
end



function t_ave_Callback(hObject, eventdata, handles)
% hObject    handle to t_ave (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of t_ave as text
%        str2double(get(hObject,'String')) returns contents of t_ave as a double


% --- Executes during object creation, after setting all properties.
function t_ave_CreateFcn(hObject, eventdata, handles)
% hObject    handle to t_ave (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function offset_Callback(hObject, eventdata, handles)
% hObject    handle to offset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of offset as text
%        str2double(get(hObject,'String')) returns contents of offset as a double


% --- Executes during object creation, after setting all properties.
function offset_CreateFcn(hObject, eventdata, handles)
% hObject    handle to offset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
