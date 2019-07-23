function plotYDotDivided(rootPath, robotName, ydotName)

legFontSize = 19;
titleFontSize = 18;
ylabFontSize = 17;
xlabFontSize = 17;

yDotstr = strcat( '/', ydotName, '.txt');
yDot = importMatrices(strcat(rootPath, robotName, yDotstr));
nRow = size(yDot, 1);
nStep = size(yDot, 3);

%millisecond indicated in missionManager
global sControlLoop
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

% plot joint commands
figure('Renderer', 'painters', 'Position', [0 0 750 990])
subplot(3,1,1);

hold on;
for i = 1:4
    a(:) = yDot(i,1,:);
    plot(seconds, a);
end
xlab = xlabel('time [s]');
leg = legend('$\dot{q}_1$','$\dot{q}_2$', '$\dot{q}_3$', '$\dot{q}_4$');
ylab = ylabel('Joint velocities [rad/s]');
if strcmp(robotName, 'g500_A')
  tq = title('Joint command for Robot A');
else
  tq = title('Joint command for Robot B');
end

set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (tq, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set (ylab, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
set (xlab, 'Interpreter', 'latex', 'FontSize', xlabFontSize);


%% plot vehicle command divided in angle e lin

subplot(3,1,2);
hold on;
for i = 5:7
    a(:) = yDot(i,1,:);
    plot(seconds, a);
end
hold off;
xlab1 = xlabel('time [s]');
ylab1 = ylabel('Linear velocity [m/s]');
if strcmp(robotName, 'g500_A')
  tq1 = title('Vehicle linear velocity command for Robot A');
else
  tq1 = title('Vehicle linear velocity command for Robot B');
end
leg1 = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$');
set(leg1, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (tq1, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set (ylab1, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
set (xlab1, 'Interpreter', 'latex', 'FontSize', xlabFontSize);

subplot(3,1,3);
hold on;
for i = 8:10
    a(:) = yDot(i,1,:);
    plot(seconds, a);
end
hold off;
xlab2 = xlabel('time [s]');
ylab2 = ylabel('Angular velocity [rad/s]');
if strcmp(robotName, 'g500_A')
  tq2 = title('Vehicle angular velocity command for Robot A');
else
  tq2 = title('Vehicle angular velocity command for Robot B');
end
leg2 = legend('$w_x$', '$w_y$', '$w_z$');
set(leg2, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (tq2, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set (ylab2, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
set (xlab2, 'Interpreter', 'latex', 'FontSize', xlabFontSize);