clear, clc, close all;

global alpha ;
alpha = 0.06;

options = odeset('RelTol',1e-4,'AbsTol',[1e-5 1e-5 1e-5 1e-5]);
[T1,Y1] = ode45(@Theta,[0 1],[0 0.5 0 0.5],options);

writerObj = VideoWriter('model.avi');
open(writerObj);

% Angla max entre les deux leviers (en degrés)
max(abs(Y1(:,1)))*180/pi;


[n1,p] = size(T1);

% Affichage des deux leviers
for i=1:10:n1
    plot(-0.1,-0.1);
    hold on;
    plot(1,1);
    plot([0,cos(Y1(i,3)-Y1(i,1))],[0,sin(Y1(i,3)-Y1(i,1))],'red');
    plot([0,cos(Y1(i,3))],[0,sin(Y1(i,3))],'black-.');
    axis equal;
    frame = getframe;
    writeVideo(writerObj,frame);
    hold off;
end;

close(writerObj);

% Angles au cours du temps
plot(T1,(Y1(:,3)-Y1(:,1)),'r-',T1,Y1(:,3),'b-.');

