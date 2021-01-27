A2=[20 30 40 50 60 70 80 90 100 110 120 130 140 150];
B2=[6.09 8.58 11.51 14.12 16.95 19.68 22.76 25.55 28.43 31.12 33.96 36.91 39.64 42.71];
C2=polyfit(A2,B2,2);
figure(1);
plot(A2,B2)
hold on
P2=poly2sym(C2);
fplot(P2);
title('Fitting of time and distance values');
xlabel('Distance(cm)');
ylabel('Time(millisecs)');
hold off;