function plotTransformMatrices(rootPath, coordName)

holeDim = 0.14;

legFontSize = 13;
titleFontSize = 16;
ylabFontSize = 15;

fileName = "wTt.txt";
wTt = importMatrices(strcat(rootPath, coordName, fileName));
fileNamegoal = "wTgoal.txt";
wTgoal = importMatrices(strcat(rootPath, coordName, fileNamegoal));
nStep = min(size(wTgoal, 3), size(wTt, 3));

%w_yLimPos =  wTt(1:3,1:3,1) * [0; holeDim/2; 0]
%w_yLimNeg =  wTt(1:3,1:3,1) * [0; -holeDim/2; 0]
%w_zLimPos =  wTt(1:3,1:3,1) * [0; 0; holeDim/2]
%w_zLimNeg =  wTt(1:3,1:3,1) * [0; 0; -holeDim/2]


global sControlLoop
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

toolLin = wTt(1:3, 4, :);
toolLinSqueezed = squeeze(toolLin(1:3,:,:));
toolAng = zeros(3, nStep);
for i = 1:nStep
  toolAng(1:3,i) = rotm2eul( wTt(1:3,1:3,i), 'ZYX');
end

goalLin = wTgoal(1:3, 4, :);
goalLinSqueezed = squeeze(goalLin(1:3,:,:));
goalAng = zeros(3, nStep);
for i = 1:nStep
  goalAng(1:3,i) = rotm2eul( wTgoal(1:3,1:3,i), 'ZYX');
end

figure
subplot(2,3,1);
plot(seconds, toolLinSqueezed(1,:));
hold on;
plot(seconds, goalLinSqueezed(1,:));
hold off;
xlabel('time [s]');
ylab1 = ylabel('tool and goal position [m]');
tq1 = title("Position Linear of goal and tool");
leg1 = legend('$x_tool$', '$x_goal$');

subplot(2,3,2);
plot(seconds, toolLinSqueezed(2,:));
hold on;
plot(seconds, goalLinSqueezed(2,:));
hold off;
xlabel('time [s]');
ylab1 = ylabel('tool and goal position [m]');
tq1 = title("Position Linear of goal and tool");
leg1 = legend('$y_tool$', '$y_goal$');

subplot(2,3,3);
plot(seconds, toolLinSqueezed(3,:));
hold on;
plot(seconds, goalLinSqueezed(3,:));
hold off;
xlabel('time [s]');
ylab1 = ylabel('tool and goal position [m]');
tq1 = title("Position Linear of goal and tool");
leg1 = legend('$z_tool$', '$z_goal$');


subplot(2,3,[4,5,6]);
plot(seconds, toolAng);
hold on
plot(seconds, goalAng);
hold off;
xlabel('time [s]');
ylab2 = ylabel('tool and goal angular position  [rad]');
tq2 = title("Angular position of tool and goal");
leg2 = legend('$roll_tool$','$pitch_tool$', '$yaw_tool$', '$roll_goal$', '$pitch_goal$', '$yaw_goal$');


set(ylab1, 'Interpreter', 'none', 'FontSize' , ylabFontSize);
set (tq1, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set(ylab2, 'Interpreter', 'none', 'FontSize' , ylabFontSize);
set (tq2, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set(leg1, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set(leg2, 'Interpreter', 'latex', 'FontSize' , legFontSize);

hold off


