clear all
close all
clc

aoa = 0:9:90;
aoa = deg2rad(aoa);
aoa = aoa';

Cd = [0.402336989
0.527045227
0.6942129
0.833964028
0.983358433
1.174163881
1.42902662
1.816267661
2.133502048
2.544393791
2.51477873
];

Cl =[0.577200052
0.885523752
1.049901706
1.043103711
1.234573746
1.170627261
1.225829899
1.237849234
1.085582808
0.946981387
0.556713837
]; 

%then, use matlab curve fitting tool in command windows
% cftool(aoa,Cd);    
% cftool(aoa,Cl);


%plot parameters:
tic1 = 0.3;
i1 =0:1:10;
y1_ticks_array = tic1*i1;

tic2 = 0.1;
i2 = 4:1:14;
y2_ticks_array = tic2*i2;

%Plot:
%cd fitting curve plot:
figure(1)
aoa_rad = deg2rad(0):deg2rad(0.1):deg2rad(90);
cd_curve = 1.35* cos( -1.376*aoa_rad + 3.08) + 1.807;

plot(aoa_rad,cd_curve,'color','r','LineWidth',2);
hold on;
grid on;
% scatter(aoa ,Cd ,80,'MarkerEdgeColor','k','MarkerFaceColor','r');
yticks(y1_ticks_array);
ylim([0,3])
% xlabel('AOA(rad)','FontSize',15);
% ylabel('Cd','FontSize',15);
% set(gca,'linewidth',2,'fontsize',40,'fontname','Times');
% legend('Cd','FontSize',20);
% title('Cd curve','FontSize',40);

%cl fitting curve plot:
% figure(2)
% aoa_rad = deg2rad(0):deg2rad(0.1):deg2rad(90);
cl_curve = 0.6551 * sin( 1.937*aoa_rad) + 0.6109;

plot(aoa_rad,cl_curve,'color','b','LineWidth',2);
% hold on;
% grid on;
scatter(aoa ,Cd ,80,'MarkerEdgeColor','k','MarkerFaceColor','r');
scatter(aoa ,Cl ,80,'MarkerEdgeColor','k','MarkerFaceColor','b');
% yticks(y2_ticks_array);
% ylim([0.4,1.4])
xlabel('AOA(rad)','FontSize',15);
ylabel('Cd & Cl','FontSize',15);
set(gca,'linewidth',2,'fontsize',40,'fontname','Times');
legend('Cd','Cl','FontSize',20);
title('Cd & Cl curve','FontSize',40);


