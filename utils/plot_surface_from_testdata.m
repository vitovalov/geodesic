
clc;clear all;close all;

suma = zeros(5,5);
%     indexar(row,column)

dd = ['bab1 '; 'bab2 '; 'Inta1';'Inta4';'vit  ';'bengu';'ir2  ';'ir3  ']
ss = cellstr(dd)
a=0;
b=0;
for name=1:8
    s = char(ss(name))
    for  i=0.01:0.01:0.05
            if a >= 5
                    a = 1;
                else
                    a=a+1
                end
            for j=5:5:25

                if b >= 5
                    b = 1;
                else
                    b=b+1
                end 
                str = sprintf('.......geodesic/processing_data/full test to plot/%s_difcoord_%.2f_%.2f.txt',s,i,j);
                str
                f = fopen(str, 'r');
                A = textscan(f, '%f %f %f %f %f', 'Delimiter', ' ') ;
                suma(b,a) = suma(b,a) + sum(A{1,5})
                fclose(f);
            end
    end 
end
Z = suma
gtruth = 120+614+226+212+293+468+238
percentageAccuracy = suma / 2171 * 100
[X,Y] = meshgrid(.01:.01:.05, 5:5:25);                                
% Z = [0, 0, 0, 0, 0; 25 15 151 255 0; 1,1,0,0,0; 1,1,1,0,0; 0, 0, 0, 1,1];  
figure(1);
surf(X,Y,Z)
% figure(2);
% surf(X,Y,Z,'FaceColor','red','EdgeColor','none')
% camlight left; lighting phong

