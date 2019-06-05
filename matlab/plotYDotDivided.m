function plotYDotDivided(rootPath, robotName, ydotName)

legFontSize = 13;
titleFontSize = 16;
ylabFontSize = 15;

yDotstr = strcat( '/', ydotName, '.txt');
yDot = importMatrices(strcat(rootPath, robotName, yDotstr));
nRow = size(yDot, 1);
nStep = size(yDot, 3);

%millisecond indicated in missionManager
global sControlLoop
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

% plot joint commands
figure
hold on;
for i = 1:4
    a(:) = yDot(i,1,:);
    plot(seconds, a);
end
xlabel('time [s]');
leg = legend('$\dot{q}_1$','$\dot{q}_2$', '$\dot{q}_3$', '$\dot{q}_4$');
ylab = ylabel('Joint command [rad/s]');
tq = title(strcat(robotName, ' Joint command sent (' ,ydotName,  '[1:4])'));
set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (tq, 'Interpreter', 'none', 'FontSize' , titleFontSize);
set (ylab, 'Interpreter', 'none', 'FontSize', ylabFontSize);
hold off

%% plot vehicle command divided in angle e lin
titleFontSize = 13; 
ylabFontSize = 12;

figure
subplot(2,1,1);
hold on;
for i = 5:7
    a(:) = yDot(i,1,:);
    plot(seconds, a);
end
hold off;
xlabel('time [s]');
ylab1 = ylabel('linear velocity [m/s]');
tq1 = title(strcat(robotName, ' Linear velocity sent to vehicle (', ydotName, '[5:7])'));
leg1 = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$');
set(leg1, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (tq1, 'Interpreter', 'none', 'FontSize' , titleFontSize);
set (ylab1, 'Interpreter', 'none', 'FontSize', ylabFontSize);

subplot(2,1,2);
hold on;
for i = 8:10
    a(:) = yDot(i,1,:);
    plot(seconds, a);
end
hold off;
xlabel('time [s]');
ylab2 = ylabel('angular velocity [rad/s]');
tq2 = title(strcat(robotName, ' Angular velocity sent to vehicle (', ydotName, '[8:10])'));
leg2 = legend('$w_x$', '$w_y$', '$w_z$');
set(leg2, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (tq2, 'Interpreter', 'none', 'FontSize' , titleFontSize);
set (ylab2, 'Interpreter', 'none', 'FontSize', ylabFontSize);