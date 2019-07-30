function plotYDotDivided2(rootPath, robotName, ydotName)

legFontSize = 19;
titleFontSize = 18;
ylabFontSize = 17;
xlabFontSize = 17;

yDotstr = strcat( '/', ydotName, '.txt');
yDot = importMatrices(strcat(rootPath, robotName, yDotstr));
if strcmp(ydotName, "yDotFinalWithCollision")
  yDotNoCol = importMatrices(strcat(rootPath, robotName, "/yDotFinal.txt"));
  yDot = yDot - yDotNoCol;

end
nStep = size(yDot, 3);

%millisecond indicated in missionManager
global sControlLoop
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

global secInsertion


figure('Renderer', 'painters', 'Position', [0 0 750 990])
%% plot joint commands
subplot(3,2,1);

hold on;
for i = 1:4
    a(:) = yDot(i,1,:);
    plot(seconds, a);
end
plot([secInsertion; secInsertion], [-0.06,0.03]', '--m');
text([secInsertion+2], [0.024], {'\rightarrow Inside the hole'}, 'Color', 'magenta', 'FontSize',14);
text(0.35,1.2,'column 1');

hold off;
xlab = xlabel('time [s]');
leg = legend('$\dot{q}_1$','$\dot{q}_2$', '$\dot{q}_3$', '$\dot{q}_4$');
ylab = ylabel('Joint velocities [rad/s]');
if strcmp(robotName, 'g500_A')
  tq = title('Joint command for Robot A');
  text(0.35,1.2,'column 1');
else
  tq = title('Joint command for Robot B');
end
ylim([-0.06,0.03]);

set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (ylab, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
set (xlab, 'Interpreter', 'latex', 'FontSize', xlabFontSize);
set (tq, 'Interpreter', 'latex', 'FontSize' , titleFontSize);


subplot(3,2,2);
hold on;
for i = 1:4
    a(:) = yDot(i,1,:);
    plot(seconds, a);
end
plot([secInsertion; secInsertion], [-0.06,0.03]', '--m');
text([secInsertion+2], [0.024], {'\rightarrow Inside the hole'}, 'Color', 'magenta', 'FontSize',14);
hold off;
xlab = xlabel('time [s]');
leg = legend('$\dot{q}_1$','$\dot{q}_2$', '$\dot{q}_3$', '$\dot{q}_4$');

ylim([-0.06,0.03]);


set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (ylab, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
set (xlab, 'Interpreter', 'latex', 'FontSize', xlabFontSize);

%% plot vehicle command divided lin

subplot(3,2,3);
hold on;
for i = 5:7
    a(:) = yDot(i,1,:);
    plot(seconds, a);
end
plot([secInsertion; secInsertion], [-0.02, 0.12]', '--m');
text([secInsertion+2], [0.11], {'\rightarrow Inside the hole'}, 'Color', 'magenta', 'FontSize',14);
hold off;
xlab1 = xlabel('time [s]');
ylab1 = ylabel('Linear velocity [m/s]');

if strcmp(robotName, 'g500_A')
  tq1 = title('Joint command for Robot A');
else
  tq1 = title('Joint command for Robot B');
end
leg1 = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$');
set(leg1, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (ylab1, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
set (xlab1, 'Interpreter', 'latex', 'FontSize', xlabFontSize);
set (tq1, 'Interpreter', 'latex', 'FontSize' , titleFontSize);

ylim([-0.02, 0.12]);


 
subplot(3,2,4);
hold on;
for i = 5:7
    a(:) = yDot(i,1,:);
    plot(seconds, a);
end
plot([secInsertion; secInsertion], [-0.02, 0.12]', '--m');
text([secInsertion+2], [0.11], {'\rightarrow Inside the hole'}, 'Color', 'magenta', 'FontSize',14);
hold off;
xlab1 = xlabel('time [s]');
ylim([-0.02, 0.12]);



leg1 = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$');
set(leg1, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (ylab1, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
set (xlab1, 'Interpreter', 'latex', 'FontSize', xlabFontSize);


%% plot vehicle command divided ang

subplot(3,2,5);
hold on;
for i = 8:10
    a(:) = yDot(i,1,:);
    plot(seconds, a);
end
plot([secInsertion; secInsertion], [-0.005, 0.045]', '--m');
text([secInsertion+2], [0.041], {'\rightarrow Inside the hole'}, 'Color', 'magenta', 'FontSize',14);
hold off;
xlab2 = xlabel('time [s]');
ylab2 = ylabel('Angular velocity [rad/s]');
ylim([-0.005, 0.045])

if strcmp(robotName, 'g500_A')
  tq2 = title('Joint command for Robot A');
else
  tq2 = title('Joint command for Robot B');
end

leg2 = legend('$w_x$', '$w_y$', '$w_z$');
set(leg2, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (ylab2, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
set (xlab2, 'Interpreter', 'latex', 'FontSize', xlabFontSize);
set (tq2, 'Interpreter', 'latex', 'FontSize' , titleFontSize);

subplot(3,2,6);
hold on;
for i = 8:10
    a(:) = yDot(i,1,:);
    plot(seconds, a);
end
plot([secInsertion; secInsertion], [-0.005, 0.045]', '--m');
text([secInsertion+2], [0.041], {'\rightarrow Inside the hole'}, 'Color', 'magenta', 'FontSize',14);
hold off;
xlab2 = xlabel('time [s]');
ylim([-0.005, 0.045])

leg2 = legend('$w_x$', '$w_y$', '$w_z$');
set(leg2, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (ylab2, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
set (xlab2, 'Interpreter', 'latex', 'FontSize', xlabFontSize);
