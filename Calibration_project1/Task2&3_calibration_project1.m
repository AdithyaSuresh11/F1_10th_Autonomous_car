x2=[20 30 40 50 60 70 80 90 100 110 120 130 140 150];
t2=[5.62 8.5 11.34 14.04 17 19.98 22.75 25.61 28.51 31.4 34.12 37.18 39.84 42.57];
z2=polyfit(x2,t2,2);
figure(1);
plot(x2,t2)
hold on
p2=poly2sym(z2);
fplot(p2);
title('Fitting of time and distance values');
xlabel('Distance(cm)');
ylabel('Time(millisecs)');
hold off;