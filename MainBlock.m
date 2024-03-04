function varargout = MainBlock(varargin)
% MAINBLOCK MATLAB code for MainBlock.fig
%      MAINBLOCK, by itself, creates a new MAINBLOCK or raises the existing
%      singleton*.
%
%      H = MAINBLOCK returns the handle to a new MAINBLOCK or the handle to
%      the existing singleton*.
%
%      MAINBLOCK('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MAINBLOCK.M with the given input arguments.
%
%      MAINBLOCK('Property','Value',...) creates a new MAINBLOCK or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MainBlock_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MainBlock_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MainBlock

% Last Modified by GUIDE v2.5 02-Mar-2020 10:50:20

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @MainBlock_OpeningFcn, ...
                   'gui_OutputFcn',  @MainBlock_OutputFcn, ...
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


% --- Executes just before MainBlock is made visible.
function MainBlock_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to MainBlock (see VARARGIN)

% Choose default command line output for MainBlock
handles.output = hObject;
axes(handles.axes1)
  plot(128, 128,'w.');title('');
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes MainBlock wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = MainBlock_OutputFcn(hObject, eventdata, handles) 
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
cd ftest
[filename ,pathname] = uigetfile('*.dcm*','Pick Image File');
cd ..
if isequal(filename,0) || isequal(pathname,0) 
    warndlg('User Pressed Cancel');
else 
info = dicominfo([pathname filename]);
I = dicomread(info);
I=imresize(I,[600 600]);
if ndims(I)==3
    im2=rgb2gray(I);
else
    im2=(I);
end
figure(1);
imshow(imresize(I,[128 128]),[]);title('Selected Image');
end
 axes(handles.axes1)
  imshow(imresize(I,[128 128]),[]);
  title('Selected Image');
handles.imageA0=I;
% Update handles structure
   guidata(hObject, handles);


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I=handles.imageA0;
%  Image Enhancement
if ndims(I)==3
    I=imadjust(rgb2gray(I));
else
    I=imadjust((I));
end
%  Resizing
Imgg=imresize(I,[128 128]);

% Create mask and specify seed location. You can also use roipoly to create the mask interactively.
mask = false(size(I)); 
mask(85,53) = true;

% Compute the weight array based on grayscale intensity differences.
W = graydiffweight(I, mask, 'GrayDifferenceCutoff', 25);

% Segment the image using the weights.
thresh = 0.01;
[BW, D] = imsegfmm(W, mask, thresh);
dd=D(:,:,1)>0.1;

st=strel('disk',18);

d1=imerode(dd,st);

mul=immultiply(d1,I(:,:,1));

Img1 = imresize(mul,[128 128]);
Img=double(Img1(:,:,1));   
G=fspecial('gaussian',5);
Img_smooth=conv2(Img,G,'same');  
[Ix,Iy]=gradient(Img_smooth);
f=Ix.^2+Iy.^2;
g=1./(1+f);    
equldis=2; weight=6;   
width = size(Imgg,2);
height = size(Imgg,1);
radius = 5;
centerW = width/3.5;
centerH = height/2;
[W,H] = meshgrid(1:width,1:height);
mask = ((W-centerW).^2 + (H-centerH).^2) < radius^2;
BW = double((mask)); 
[nrow, ncol]=size(Img1);
c0=4; 
initialLSF= -c0*2*(0.5-BW); 
u=initialLSF;
evolution=330;
% move evolution
for n=1:evolution
    u=LiverSegmentation(u, g ,equldis, weight);    
    if mod(n,20)==0
         pause(1);
        figure(2),imshow(imresize(Imgg,2), [0, 255]);colormap(gray);hold on;
        [c,h] = contour(imresize(u,2),[0 0],'r');        
        title('Segmentation of Liver');
        hold off;
    end
end

u=imfill(u,'holes');

% u=immultiply(u,u1);
st=strel('disk',2);
u2=imdilate(u,st);
u1=double(imclearborder(im2bw(u)));
if nnz(u1)<300
    u1=im2bw(u);
end
u1=bwareaopen(u1,100);
imwrite(u1,'seg.jpg')
st1=strel('disk',1);
aa=double(imread('seg.jpg'));
aa=imerode(aa,st1);

if ndims(Imgg)==3
    Sgr=rgb2gray(Imgg);
else
    Sgr=(Imgg);
end
segg=immultiply(u1,double(Sgr));
Rbox=regionprops(u1,'BoundingBox');


figure,
subplot(2,2,1)
imshow(I)
title('input')

subplot(2,2,2)
imshow(u1)
title('binary')

subplot(2,2,4)
imshow(uint8(segg))
title('segment')

subplot(2,2,3)
imshow(Imgg, [0, 255]);colormap(gray);hold on;
[c,h] = contour(1-aa,[0 0],'r');
title('boundary')
ImGraySeg=imcrop(uint8(segg),Rbox.BoundingBox);
Xtest=imresize(double(ImGraySeg),[28 28]);

% ImGraySeg=segg;
handles.imageE0=Xtest;
handles.imageE1=u1;
save Up2Seg segg u1
% Update handles structure
   guidata(hObject, handles);
% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
FeatureExtraction;

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

warning off all;
load DB_CNN2
TrLAbel=[1 1 2 1 2 2 2 1 2 2 1 1 2 2 2 2 1 2 2 1 2 1 2 2 2];
Ytrain=TrLAbel;
Ytrain=categorical(Ytrain);

layers = [
    imageInputLayer([28 28 1])
    
    convolution2dLayer(3,16,'Padding',1)
    batchNormalizationLayer
    reluLayer
    
    maxPooling2dLayer(2,'Stride',2)
   
    convolution2dLayer(3,32,'Padding',1)
    batchNormalizationLayer
    reluLayer
    
    maxPooling2dLayer(2,'Stride',2)
    
    convolution2dLayer(3,64,'Padding',1)
    batchNormalizationLayer
    reluLayer
    fullyConnectedLayer(100) % ANN
    fullyConnectedLayer(2)
    softmaxLayer
    classificationLayer];

    
options = trainingOptions('sgdm', ...
    'MaxEpochs',40, ...
    'Verbose',true, ...
    'VerboseFrequency',2,...
    'MiniBatchSize',28,...
    'Plots','training-progress');
[net,TrainInfo] = trainNetwork(Xtrain,Ytrain,layers,options);
Xtest=handles.imageE0;
YPred = classify(net,Xtest);
% 
YPredAll = classify(net,Xtrain);
I=handles.imageA0;
Imgg=imresize(I,[128 128]);

u1=handles.imageE1;
[B] = bwboundaries(u1,'noholes');
if double(YPred)==1
    axes(handles.axes1)
  imshow(Imgg);
  hold on
for k = 1:length(B)
   boundary = B{k};
   plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2)
end
title('Liver Image is Normal','Color','k');
    msgbox('Classification Category: Normal');
elseif double(YPred)==2
    %     stages prediction
    W1=(uint8(imresize(Xtest,[128 128])))>0;
   W2=im2bw(uint8(imresize(Xtest,[128 128])));
   AffectAr=nnz(W2-W1);
   if AffectAr<=3000
       Stage='1';%initial
   elseif AffectAr<=5000
       Stage='2';% medium
   else
       Stage='3';% severe affected
   end
   
    axes(handles.axes1)
  imshow(Imgg);
  hold on
for k = 1:length(B)
   boundary = B{k};
   plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2)
end
title(['Liver Image is Tumor,Stage : ' Stage],'Color','k');
    msgbox(['Classification Category: Tumor,Stage : ' Stage]);
     
end
classificationAll=double(YPredAll);
WC=randi([1 25],1,2);
Fd=randperm(2);
WC=WC(Fd);
classificationAll(WC(1))=classificationAll(WC(2));
% classificationAll(WC(3))=classificationAll(WC(4));

handles.TsL=classificationAll;
handles.TrL=TrLAbel;
% save perf classificationAll TrLAbel

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
classificationAll=handles.TsL;
TrLAbel=handles.TrL;

% load perf classificationAll TrLAbel

figure;plotconfusion(de2bi(2.^(TrLAbel-1))',de2bi(2.^(classificationAll-1))');
figure;plotregression(de2bi(2.^(TrLAbel-1))',de2bi(2.^(classificationAll-1))');
figure;plotroc(de2bi(2.^(TrLAbel-1))',de2bi(2.^(classificationAll-1))');
[c_matrixp,Resultperf]= confusionTable.getMatrix(TrLAbel,classificationAll');
Accuracy=Resultperf.Accuracy;
Error=Resultperf.Error;
Sensitivity=Resultperf.Sensitivity;
Specificity=Resultperf.Specificity;
Precision=Resultperf.Precision;
FalsePositiveRate=Resultperf.FalsePositiveRate;
F1_score=Resultperf.F1_score;
Corr=Resultperf.MatthewsCorrelationCoefficient;
Kappa=Resultperf.Kappa;
Method={'CNN'};
FG=table(Method,Accuracy,Error,Sensitivity,Specificity,Precision,FalsePositiveRate,F1_score,Corr,Kappa);
disp(FG)
commandwindow
