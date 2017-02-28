clc;clear all;close all;



for i=673:674
    str = sprintf('txt/meanVec-%d.txt',i);
    f=fopen(str);
    A = fscanf(f, '%f');
    fig=figure(i);
    plot(A);
    namesave = sprintf('plot-%d.png',i);
    saveas(fig,namesave);
    
end

f=fopen('txt/finalVec.txt');
A = fscanf(f, '%f');
fig = figure(2);
plot(A);
saveas(fig,'finalVec.png');