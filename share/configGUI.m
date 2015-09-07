function varargout = configGUI(varargin)
% CONFIGGUI MATLAB code for configGUI.fig
%      CONFIGGUI, by itself, creates a new CONFIGGUI or raises the existing
%      singleton*.
%
%      H = CONFIGGUI returns the handle to a new CONFIGGUI or the handle to
%      the existing singleton*.
%
%      CONFIGGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CONFIGGUI.M with the given input arguments.
%
%      CONFIGGUI('Property','Value',...) creates a new CONFIGGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before configGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to configGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help configGUI

% Last Modified by GUIDE v2.5 22-Apr-2015 09:28:25

% Date: 2013-04-28
% Author: Zihan Chen

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @configGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @configGUI_OutputFcn, ...
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


% --- Executes just before configGUI is made visible.
function configGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to configGUI (see VARARGIN)

% Choose default command line output for configGUI
handles.output = hObject;
handles.m_cal_filename = '';  % calibration file
handles.m_out_filename = '';  % output xml file

% set default board id
handles.m_boardID(1) = 0;
handles.m_boardID(2) = 1;
set(handles.bidPopup1,'Value', handles.m_boardID(1) + 1);
set(handles.bidPopup2,'Value', handles.m_boardID(2) + 1);

% set default type
handles.m_type = 'MTML';
set(handles.typeBtnGroup, 'SelectedObject', handles.mtmlButton);
handles.m_out_filename = ['sawRobotIO1394-' handles.m_type];
set(handles.out_name, 'String', handles.m_out_filename);

% set default digitalIn data
handles.m_digiIn = cell(12, 4, 2);
for i = 1:12
    handles.m_digiIn{i,1,1} = i-1;
    handles.m_digiIn{i,2,1} = [handles.m_type, '-D', ...
                              num2str(idivide(i-1, int32(4))), ...
                              num2str(mod(i-1,4)) ];
    handles.m_digiIn{i,3,1} = '0';
    handles.m_digiIn{i,4,1} = 'all';
    handles.m_digiIn{i,5,1} = 0.0;

    handles.m_digiIn{i,1,2} = i-1;
    handles.m_digiIn{i,2,2} = [handles.m_type, '-D', ...
                              num2str(idivide(i-1, int32(4))+3), ...
                              num2str(mod(i-1,4)) ];
    handles.m_digiIn{i,3,2} = '0';
    handles.m_digiIn{i,4,2} = 'all';
    handles.m_digiIn{i,5,2} = 0.0;
end
set(handles.digiBidMenu, 'Value', 1);
set(handles.tblDigital, 'data', handles.m_digiIn(:,:,1));

% set default drive direction data
handles.m_direction = cell(8,1);
for i = 1:8
    handles.m_direction{i} = 1;
end
set(handles.tblDirection, 'data', handles.m_direction);

% Update handles structure
guidata(hObject, handles);


% UIWAIT makes configGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = configGUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in calButton.
function calButton_Callback(hObject, eventdata, handles)
% hObject    handle to calButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename, pathname] = uigetfile('*.cal', 'Select a da vinci calibration file');
if isequal(filename,0)
   disp('User selected Cancel')
else
   disp(['User selected ', fullfile(pathname, filename)]);
   disp(filename);
   set(handles.cal_name,'String', fullfile(pathname, filename));
   handles.m_cal_filename = fullfile(pathname, filename);
end
% save values
guidata(hObject, handles);


% --- Executes on button press in generateButton.
function generateButton_Callback(hObject, eventdata, handles)
% hObject    handle to generateButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
disp('Please generate my xml file');
disp(handles.m_cal_filename);
disp(handles.m_boardID);
disp(handles.m_type);

% updateDigitalData();

disp('Generating config file');
isOK = configGenerator(...
    handles.m_cal_filename, ...
    handles.m_out_filename, ...
    handles.m_type, ...
    handles.m_boardID, ...
    handles.m_digiIn, ...
    cell2mat(handles.m_direction)'); % change to horizontal num array
if (isOK)
    msgbox('Config file generated successfully');
else
    msgbox('ERROR: failed to generate config file');
end


% --- Executes on selection change in bidPopup1.
function bidPopup1_Callback(hObject, eventdata, handles)
% hObject    handle to bidPopup1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns bidPopup1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from bidPopup1
val = get(hObject, 'Value');
handles.m_boardID(1) = val - 1;
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function bidPopup1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bidPopup1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in bidPopup2.
function bidPopup2_Callback(hObject, eventdata, handles)
% hObject    handle to bidPopup2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns bidPopup2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from bidPopup2
val = get(hObject, 'Value');
handles.m_boardID(2) = val - 1;
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function bidPopup2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bidPopup2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes when selected object is changed in typeBtnGroup.
function typeBtnGroup_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in typeBtnGroup
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)

handles.m_type = get(hObject, 'String');
% update DigitalIn
for i = 1:12
    handles.m_digiIn{i,2,1} = [handles.m_type '-D' ...
                               num2str(idivide(i-1, int32(4))) ...
                               num2str(mod(i-1,4))];
    handles.m_digiIn{i,5,1} = 0.0;

    handles.m_digiIn{i,2,2} = [handles.m_type '-D' ...
                              num2str(idivide(i-1, int32(4))+3) ...
                              num2str(mod(i-1,4))];
    handles.m_digiIn{i,5,2} = 0.0;
end


% set default PSM DigitalIn
if (strcmp(handles.m_type,'PSM1') || strcmp(handles.m_type,'PSM2') || strcmp(handles.m_type,'PSM3'))
    handles.m_digiIn{1,2,1} = [handles.m_type '-SUJClutch'];
    handles.m_digiIn{1,5,1} = 0.2;
    handles.m_digiIn{3,2,1} = [handles.m_type '-ManipClutch'];
    handles.m_digiIn{3,5,1} = 0.2;
    handles.m_digiIn{8,2,2} = [handles.m_type '-Tool'];
    handles.m_digiIn{8,5,2} = 1.5;
    handles.m_digiIn{11,2,2} = [handles.m_type '-Adapter'];
    handles.m_digiIn{11,5,2} = 1.5;
end

if (strcmp(handles.m_type,'ECM'))
    handles.m_digiIn{1,2,1} = [handles.m_type '-ManipClutch'];
    handles.m_digiIn{1,5,1} = 0.2;
    handles.m_digiIn{3,2,1} = [handles.m_type '-SUJClutch'];
    handles.m_digiIn{3,5,1} = 0.2;
end

val = get(handles.digiBidMenu, 'Value');
set(handles.tblDigital, 'data', handles.m_digiIn(:,:,val));

% update default output file name
handles.m_out_filename = ['sawRobotIO1394-' handles.m_type];
set(handles.out_name, 'String', handles.m_out_filename);
guidata(hObject, handles);

% update default bid
handles = bidSetDefault(handles);

% update default direction
dirDefaultButton_Callback(handles.dirDefaultButton, eventdata, handles);


% --- Executes when entered data in editable cell(s) in tblDigital.
function tblDigital_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to tblDigital (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)

% handles.m_digiIn(:,:,handles.m_digiIn_bid) = get(hObject, 'data');
val = get(handles.digiBidMenu, 'Value');
if (val == 1)
    handles.m_digiIn(:,:,1) = get(handles.tblDigital, 'data');
elseif (val == 2)
    handles.m_digiIn(:,:,2) = get(handles.tblDigital, 'data');
end

disp(handles.m_digiIn);
guidata(hObject, handles);


% --- Executes on selection change in digiBidMenu.
function digiBidMenu_Callback(hObject, eventdata, handles)
% hObject    handle to digiBidMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns digiBidMenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from digiBidMenu

val = get(hObject, 'Value');
set(handles.tblDigital, 'data', handles.m_digiIn(:,:,val));
% guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function digiBidMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to digiBidMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function out_name_Callback(hObject, eventdata, handles)
% hObject    handle to out_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of out_name as text
%        str2double(get(hObject,'String')) returns contents of out_name as a double


% --- Executes during object creation, after setting all properties.
function out_name_CreateFcn(hObject, eventdata, handles)
% hObject    handle to out_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% function [ newDigiData ] = updateDigitalData(oldDigiData)
% disp('updateDigiData');


% --- Update handles default bid
function handles = bidSetDefault(handles)
if (strcmp(handles.m_type,'MTML'))
    handles.m_boardID = [0 1];
elseif (strcmp(handles.m_type,'MTMR'))
    handles.m_boardID = [2 3];
elseif (strcmp(handles.m_type,'ECM'))
    handles.m_boardID = [4 5];
elseif (strcmp(handles.m_type,'PSM1'))
    handles.m_boardID = [6 7];
elseif (strcmp(handles.m_type,'PSM2'))
    handles.m_boardID = [8 9];
elseif (strcmp(handles.m_type,'PSM3'))
    handles.m_boardID = [10 11];
else
    disp('ERROR: unknown hardware type');
end
set(handles.bidPopup1, 'Value', handles.m_boardID(1) + 1);
set(handles.bidPopup2, 'Value', handles.m_boardID(2) + 1);


% --- Executes on button press in bidDefaultButton.
function bidDefaultButton_Callback(hObject, eventdata, handles)
% hObject    handle to bidDefaultButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = bidSetDefault(handles);

guidata(hObject, handles);


% --- Executes on button press in footButton.
function footButton_Callback(hObject, eventdata, handles)
% hObject    handle to footButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.m_digiIn{4,2,1} = 'HEAD';
handles.m_digiIn{4,5,1} = 0.2;
handles.m_digiIn{1,2,2} = 'CLUTCH';
handles.m_digiIn{1,5,2} = 0.2;
handles.m_digiIn{2,2,2} = 'CAM-';
handles.m_digiIn{2,5,2} = 0.2;
handles.m_digiIn{3,2,2} = 'CAM+';
handles.m_digiIn{3,5,2} = 0.2;
handles.m_digiIn{4,2,2} = 'COAG';
handles.m_digiIn{4,5,2} = 0.2;
handles.m_digiIn{5,2,2} = 'CAMERA';
handles.m_digiIn{5,5,2} = 0.2;

val = get(handles.digiBidMenu, 'Value');
set(handles.tblDigital, 'data', handles.m_digiIn(:,:,val));

% update default output file name
handles.m_out_filename = ['sawRobotIO1394-' handles.m_type '-foot-pedal'];
set(handles.out_name, 'String', handles.m_out_filename);
guidata(hObject, handles);


% --- Executes on button press in dirDefaultButton.
function dirDefaultButton_Callback(hObject, eventdata, handles)
% hObject    handle to dirDefaultButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% NOTE: this default is based on JHU's hardware, you might have
% different drive direction
if (strcmp(handles.m_type,'MTML'))
    handles.m_direction = {-1; 1; 1; 1; -1; 1; -1; 1};
elseif (strcmp(handles.m_type,'MTMR'))
    handles.m_direction = {-1; 1; 1; 1; 1; 1; -1; 1};
elseif (strcmp(handles.m_type,'PSM1') || strcmp(handles.m_type,'PSM2') || strcmp(handles.m_type,'PSM3'))
    handles.m_direction = {-1; -1; 1; -1; -1; 1; 1; 1};
elseif (strcmp(handles.m_type,'ECM'));
    handles.m_direction = {1; 1; -1; 1};
end
set(handles.tblDirection, 'data', handles.m_direction);
guidata(hObject, handles);


% --- Executes when entered data in editable cell(s) in tblDirection.
function tblDirection_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to tblDirection (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)
handles.m_direction = get(handles.tblDirection, 'data');
disp(handles.m_direction);
guidata(hObject, handles);
