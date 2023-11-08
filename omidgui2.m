function varargout = omidgui2(varargin)
% OMIDGUI2 MATLAB code for omidgui2.fig
%      OMIDGUI2, by itself, creates a new OMIDGUI2 or raises the existing
%      singleton*.
%
%      H = OMIDGUI2 returns the handle to a new OMIDGUI2 or the handle to
%      the existing singleton*.
%
%      OMIDGUI2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in OMIDGUI2.M with the given input arguments.
%
%      OMIDGUI2('Property','Value',...) creates a new OMIDGUI2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before omidgui2_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to omidgui2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help omidgui2

% Last Modified by GUIDE v2.5 17-Jan-2018 03:03:33

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @omidgui2_OpeningFcn, ...
    'gui_OutputFcn',  @omidgui2_OutputFcn, ...
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


% --- Executes just before omidgui2 is made visible.
function omidgui2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to omidgui2 (see VARARGIN)

% Choose default command line output for omidgui2
handles.output = hObject;
handles.f=get(handles.FR,'Value');
set(handles.E2,'string',num2str(handles.f));
handles.time = str2double(get(handles.edit3,'String'));
guidata(hObject, handles);


% UIWAIT makes omidgui2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = omidgui2_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes during object creation, after setting all properties.
function figure1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called








% --- Executes during object creation, after setting all properties.
function axes2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes2


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.Kr=get(hobject,'value');
set(handles.Mr,'string',num2str(handles.Kr));
guidata(hobject,handles);


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



% --- Executes during object creation, after setting all properties.
function text2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



function Mr_Callback(hObject, eventdata, handles)
% hObject    handle to Mr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Mr as text
%        str2double(get(hObject,'String')) returns contents of Mr as a double


% --- Executes during object creation, after setting all properties.
function Mr_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Mr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function FR_Callback(hObject, eventdata, handles)
% hObject    handle to FR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.f=get(handles.FR,'Value');
set(handles.E2,'string',num2str(handles.f));
guidata(hObject, handles);

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider



% --- Executes during object creation, after setting all properties.
function FR_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function E2_Callback(hObject, eventdata, handles)
% hObject    handle to E2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.f=str2double(get(hObject,'string'));
set(handles.FR,'value',handles.f);
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of E2 as text
%        str2double(get(hObject,'String')) returns contents of E2 as a double

% --- Executes during object creation, after setting all properties.
function E2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to E2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
set(hObject,'string',num2str(500));






function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double
handles.time = str2double(get(hObject,'String'))
guidata(hObject,handles);



% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
set(hObject,'string',num2str(0));


% --- Executes on button press in radiobutton4.
function radiobutton4_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.ma= get(hObject,'Value');

guidata(hObject,handles);


% Hint: get(hObject,'Value') returns toggle state of radiobutton4


% --- Executes on button press in radiobutton5.
function radiobutton5_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.noisecl= get(hObject,'Value');
guidata(hObject,handles);

% Hint: get(hObject,'Value') returns toggle state of radiobutton5
function radiobutton7_Callback(hObject, eventdata, handles)
handles.z= get(hObject,'Value');
guidata(hObject,handles);


% --- Executes on button press in radiobutton8.
function radiobutton8_Callback(hObject, eventdata, handles)
handles.z1= get(hObject,'Value');

guidata(hObject,handles);


% --- Executes on button press in radiobutton9.
function radiobutton9_Callback(hObject, eventdata, handles)
handles.z2= get(hObject,'Value');

guidata(hObject,handles);
% --- Executes on button press in radiobutton10.
function radiobutton10_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of radiobutton10
guidata(hObject,handles);

% --- Executes on button press in radiobutton11.
function radiobutton11_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of radiobutton11
guidata(hObject,handles);

% --- Executes on button press in radiobutton12.
function radiobutton12_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
guidata(hObject,handles);
% Hint: get(hObject,'Value') returns toggle state of radiobutton12


% --- Executes on button press in radiobutton13.
function radiobutton13_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
guidata(hObject,handles);
% Hint: get(hObject,'Value') returns toggle state of radiobutton13

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% Hint: get(hObject,'Value') returns toggle state of radiobutton13
% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if get(handles.radiobutton12,'value')==1
    fc                 = (handles.f)*1000000; % Center frequency (Hz)
    if (fc<100e6)
        errordlg('please enter frequency between 88~110MHz','warning');
    else
        FrontEndSampleRate = 2400e3;     % Samples per second
        FrameLength        = 16128;
        hSDRrRx = comm.SDRRTLReceiver(...
            'CenterFrequency', fc, ...
            'EnableTunerAGC',  true, ...
            'SampleRate',      FrontEndSampleRate, ...
            'SamplesPerFrame', FrameLength, ...
            'OutputDataType',  'double');
        if get(handles.radiobutton7,'value')==1
            handles.toolbaze=256;
        end
        if get(handles.radiobutton8,'value')==1
            handles.toolbaze=1024;
        end
        if get(handles.radiobutton9,'value')==1
            handles.toolbaze=4096;
        end
        for i=0:100
            data=step(hSDRrRx) ;
            n=handles.toolbaze;
            y = fft(data,n);
            y0 = fftshift(y);
            f0 = (-n/2:n/2-1)*(2400e3/n);
            power0 = y0.*conj(y0)/n;
            plot(fc-f0,power0)
            drawnow
        end
    end
end

if (get(handles.radiobutton10,'value')==1 && get(handles.radiobutton13,'value')==1)
    fc                 = (handles.f)*1000000; % Center frequency (Hz)
    if (fc>110e6)
        errordlg('please enter frequency between 88~110MHz','warning');
    else
        FrontEndSampleRate = 1e6;     % Samples per second
        FrameLength        = 2.^18;
        
        load ('filter_coef1.mat') ;
        load('testfilter.mat')
        hSDRrRx = comm.SDRRTLReceiver(...
            'CenterFrequency', fc, ...
            'EnableTunerAGC',  true, ...
            'SampleRate',      FrontEndSampleRate, ...
            'SamplesPerFrame', FrameLength, ...
            'OutputDataType',  'double');
        for j=0:1:handles.time
            set(handles.timerr,'string',num2str(j));
                         for i=0:3
                    handles.Ample=get(handles.slider3,'value');
                    %for playing FM
                    data1=step(hSDRrRx) ;
                    data_filtered1=filter(lowpassf,1,data1);
                    diff_at_phase1=data_filtered1.*conj([data_filtered1(2:end);0]);
                    mt1=angle(diff_at_phase1);
                    dc=mean(mt1);
                    mt1_without_dc=mt1-dc;                  
                    decrease_fs1=decimate(mt1_without_dc,22);
                    mt1_filtered1=filter(voice_filter,1,decrease_fs1);
                    sound((handles.Ample*mt1_filtered1)/max(mt1_filtered1),45454)
                    %for plotting psd2
                    x1=pwelch(data_filtered1);
                    f1=length(x1);
                    psd21 = fftshift(x1);
                    f1_1=(-f1/2:1:(f1/2)-1)*(1e6/length(x1));
                    plot(fc-f1_1,psd21)
                    drawnow       
                         end
        end
    end
end
    if (get(handles.radiobutton11,'value')==1 && get(handles.radiobutton13,'value')==1)
        fc                 = ((handles.f)*1000000-300e3); % Center frequency (Hz)
        if (fc>110e6)
            errordlg('please enter frequency between 88~110MHz','warning');
        else
            FrontEndSampleRate = 1e6;     % Samples per second
            FrameLength        = 2.^18;
            load('bpfilter.mat', 'my_bandpass_filter')
            load('bpfilter2.mat')
            load ('filter_coef1.mat') ;
            load('testfilter.mat')
            hSDRrRx = comm.SDRRTLReceiver(...
                'CenterFrequency', fc, ...
                'EnableTunerAGC',  true, ...
                'SampleRate',      FrontEndSampleRate, ...
                'SamplesPerFrame', FrameLength, ...
                'OutputDataType',  'double');
            %%for playing FM
            for j=0:1:handles.time
                set(handles.timerr,'string',num2str(j));
                          for i=0:3
                              handles.Ample=get(handles.slider3,'value');
                        data2=step(hSDRrRx) ;
                        data_filtered2=filter(my_bandpass_filter2,1,data2);
                        envelope=abs(hilbert(diff(real(data_filtered2))));
                        dc=mean((envelope));
                        mt2=envelope-dc;
                        decreased_fs2=decimate(mt2,10);
                        mt2_filtered=filter(voice_filter,1,decreased_fs2);
                        sound(handles.Ample*mt2_filtered/max(mt2_filtered),100e3)
                        %%for plotting psd
                        x2=pwelch(data_filtered2);
                        f2=length(x2);
                        f2_2=(-f2/2:f2/2-1)*(1e6/f2);
                        psd2 = fftshift(x2);
                        plot(fc-f2_2,psd2)
                        drawnow
                          end
            end
        end
    end
   

function pushbutton2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes during object creation, after setting all properties.
function uipanel1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to uipanel1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
function uipanel3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to uipanel1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.Ample=get(handles.slider3,'value');
set(handles.volume_num,'string',num2str(handles.Ample));
guidata(hObject, handles);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function volume_num_Callback(hObject, eventdata, handles)
% hObject    handle to volume_num (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.Ample=str2double(get(hObject,'string'));
set(handles.slider3,'value',handles.Ample);
guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of volume_num as text
%        str2double(get(hObject,'String')) returns contents of volume_num as a double


% --- Executes during object creation, after setting all properties.
function volume_num_CreateFcn(hObject, eventdata, handles)
% hObject    handle to volume_num (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



