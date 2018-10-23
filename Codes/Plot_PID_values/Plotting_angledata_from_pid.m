clc
clear all 

% Loading Data to be plotted
load('matlab.mat');  

f = figure ;
subplot(2,1,1);
% Plotting UnFiltered data
plot(time,data);

% Butterworth filter with 2nd order and 0.01 threshold
[b,a] = butter(2,0.01);
subplot(2,1,2);
% Plotting Filtered data
plot(time,filter(b,a,data));

% Giving titles to each subplot
ax = findobj(f,'Type','Axes');
for i=1:length(ax)
    xlabel(ax(i),{'Time'});
    ylabel(ax(i),{'Angle'});
    title(ax(i),{'p = 5, d = 0.8, i = 0.002'});
end