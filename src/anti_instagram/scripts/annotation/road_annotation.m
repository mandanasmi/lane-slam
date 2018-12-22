function varargout = road_annotation(varargin)
% ROAD_ANNOTATION MATLAB code for road_annotation.fig
%      ROAD_ANNOTATION(image_file_list), where image_file_list is a cell array of string, creates a new ROAD_ANNOTATION or raises the existing
%      singleton*.
%
%      H = ROAD_ANNOTATION(image_file_list) returns the handle to a new ROAD_ANNOTATION or the handle to
%      the existing singleton*.
%
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help road_annotation

% Last Modified by GUIDE v2.5 08-Apr-2016 17:01:30

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @road_annotation_OpeningFcn, ...
                   'gui_OutputFcn',  @road_annotation_OutputFcn, ...
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

function ud=update_image(ud)
pos=get(ud.axes1,'Position');
ud.I=imread(ud.files_list{ud.file_idx});
annotations=ud.annotations_map(ud.files_list{ud.file_idx});
ud.display_image=ud.I;
imshow(ud.display_image,[],'parent',ud.axes1)
for i = 1:numel(annotations)
    hold on;
    h=fill(annotations{i}.x,annotations{i}.y,annotations{i}.color,'FaceAlpha',0.25);
    hold off;
end
ud.filename_text.String=ud.filename;
drawnow;

% --- Executes just before road_annotation is made visible.
function road_annotation_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to road_annotation (see VARARGIN)

% Choose default command line output for road_annotation
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
ud=guidata(hObject);
ud.files_list=varargin{1};
ud.annotations_map=containers.Map;
for i = 1:numel(ud.files_list)
    ud.annotations_map(ud.files_list{i})={};
end
ud.file_idx=1;
ud.filename=ud.files_list{ud.file_idx};

%
% Set detection categories - TODO: make this
iparser = inputParser;
iparser.CaseSensitive=false;
default_classes={};
default_classes{end+1}=struct('name','road','color',[0 0 0]);
default_classes{end+1}=struct('name','red','color',[1 0 0]);
default_classes{end+1}=struct('name','white','color',[1 1 1]);
default_classes{end+1}=struct('name','yellow','color',[1 1 0]);
iparser.addParameter('classes',default_classes);
iparser.parse(varargin{2:end});

ud.colors={};
ud.listbox1.String={};
for i = 1:numel(iparser.Results.classes)
    ud.colors{end+1}=iparser.Results.classes{i}.color;
    ud.listbox1.String{end+1}=iparser.Results.classes{i}.name;
end

ud=update_image(ud);

guidata(hObject,ud);

% UIWAIT makes road_annotation wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = road_annotation_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton_prev_file.
function pushbutton_prev_file_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_prev_file (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
guidata(hObject, handles);
ud=guidata(hObject);
ud.file_idx=max(1,ud.file_idx-1);
ud.filename=ud.files_list{ud.file_idx};
ud=update_image(ud);
guidata(hObject,ud);


% --- Executes on button press in pushbutton_next_file.
function pushbutton_next_file_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_next_file (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ud=guidata(hObject);
ud.file_idx=min(numel(ud.files_list),ud.file_idx+1);
ud.filename=ud.files_list{ud.file_idx};
ud=update_image(ud);
guidata(hObject,ud);


% --- Executes on selection change in listbox1.
function listbox1_Callback(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox1


% --- Executes during object creation, after setting all properties.
function listbox1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_label.
function pushbutton_label_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_label (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ud=guidata(hObject);
axes(ud.axes1)
[mask,x,y]=roipoly;
annotations=ud.annotations_map(ud.files_list{ud.file_idx});
region_type=ud.listbox1.Value;
annotations{end+1}=struct('type',region_type,'x',x,'y',y,'mask',mask,'color',ud.colors{region_type});
ud.annotations_map(ud.files_list{ud.file_idx})=annotations;
ud=update_image(ud);
guidata(hObject,ud);


% --- Executes on button press in pushbutton_save.
function pushbutton_save_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ud=guidata(hObject);
[FileName,PathName] = uiputfile('*.mat','Select save file name');
save([PathName,FileName],'ud');

% --- Executes on button press in pushbutton_clear.
function pushbutton_clear_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ud=guidata(hObject);
ud.annotations_map(ud.files_list{ud.file_idx})={};
ud=update_image(ud);
guidata(hObject,ud);
