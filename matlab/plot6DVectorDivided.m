function plot6DVectorDivided(rootPath, robotName, vecName)

legFontSize = 19;
titleFontSize = 18;
ylabFontSize = 17;
xlabFontSize = 17;

vecNamestr = strcat( '/', vecName, '.txt');
vec = importMatrices(strcat(rootPath, robotName, vecNamestr));
nStep = size(vec, 3);

%millisecond indicated in missionManager
global sControlLoop
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

% plot joint commands
figure('Renderer', 'painters', 'Position', [0 0 710 550])
subplot(2,1,1);
hold on;
for i = 1:3
    a(:) = vec(i,1,:);
    plot(seconds, a);
end
xlab = xlabel('time [s]');
leg = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$');

if strcmp(vecName, 'toolVel4Collision')
    ylab = ylabel('Linear Velocity [m/s]');
    tq = title('Tool Velocities due to Collisions');
elseif strcmp(vecName, 'toolVel4Grasp')
    ylab = ylabel('Linear Velocity [m/s]');
    tq = title('Tool Velocities due to Grasp Constraint');
else
  ylab = ylabel(' vector [?]');
  tq = title(strcat(robotName, " LINEAR "  ,vecName));
end

subplot(2,1,2);
hold on;
for i = 4:6
    a(:) = vec(i,1,:);
    plot(seconds, a);
end
xlab2 = xlabel('time [s]');

leg2 = legend('$w_{x}$','$w_{y}$', '$w_{z}$');
if strcmp(vecName, 'toolVel4Collision')
    ylab2 = ylabel('Angular Velocity [rad/s]');
    tq2 = title('Tool Velocities due to Collisions');
elseif strcmp(vecName, 'toolVel4Grasp')
    ylab2 = ylabel('Angular Velocity [rad/s]');
    tq2 = title('Tool Velocities due to Grasp Constraint');
else
	 ylab2 = ylabel(' vector [?]');
  tq2 = title(strcat(robotName, " ANGULAR "  ,vecName));
end

set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (tq, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set (ylab, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
set (xlab, 'Interpreter', 'latex', 'FontSize', xlabFontSize);

set(leg2, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (tq2, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set (ylab2, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
set (xlab2, 'Interpreter', 'latex', 'FontSize', xlabFontSize);

