function plotTransformMatrices(rootPath, coordName)

legFontSize = 19;
titleFontSize = 18;
ylabFontSize = 17;
xlabFontSize = 17;


wLinGReal = [1.02545;  -10.1984;  8.37884];
                    
wAngGReal = [0; 0; -1.443185307179587];

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

global secInsertion

toolLin = wTt(1:3, 4, :);
toolLinSqueezed = squeeze(toolLin(1:3,:,:));
toolAng = zeros(3, nStep);
for i = 1:nStep
  toolAng(1:3,i) = rotm2eul( wTt(1:3,1:3,i), 'XYZ');
end

goalLin = wTgoal(1:3, 4, :);
goalLinSqueezed = squeeze(goalLin(1:3,:,:));
goalAng = zeros(3, nStep);
for i = 1:nStep
  goalAng(1:3,i) = rotm2eul( wTgoal(1:3,1:3,i), 'XYZ');
end

%%
figure('Renderer', 'painters', 'Position', [0 0 2000 900])
%% plot X lin
subplot(2,3,1);
plot(seconds, toolLinSqueezed(1,:));
hold on;
plot(seconds, goalLinSqueezed(1,:));
plot([seconds(1), seconds(end)], [wLinGReal(1), wLinGReal(1)]);
%vertical line to show where insertion begin
plot([secInsertion; secInsertion], [ylim]', '--m');
text([secInsertion+2], [1.07], {'\rightarrow Inside the hole'}, 'Color','magenta', 'FontSize',14);
hold off;
xlab1 = xlabel('time [s]');
ylab1 = ylabel('Vectors (X) [m]');
tq1 = title("X Position of tool, goal and goalReal");
leg1 = legend('$x_{tool}$', '$x_{goal}$', '$x_{goalReal}$');


%% plot y lin
subplot(2,3,2);
plot(seconds, toolLinSqueezed(2,:));
hold on;
plot(seconds, goalLinSqueezed(2,:));
plot([seconds(1), seconds(end)], [wLinGReal(2), wLinGReal(2)]);
plot([secInsertion; secInsertion], [ylim]', '--k');
%vertical line to show where insertion begin
plot([secInsertion; secInsertion], [ylim]', '--m');
text([secInsertion+2], [-5.32], {'\rightarrow Inside the hole'}, 'Color','magenta', 'FontSize',14);
hold off;
xlab2 = xlabel('time [s]');
ylab2 = ylabel('Vectors (Y) [m]');
tq2 = title("Y Position of tool, goal and goalReal");
leg2 = legend('$y_{tool}$', '$y_{goal}$', '$y_{goalReal}$');

%% plot z lin
subplot(2,3,3);
plot(seconds, toolLinSqueezed(3,:));
hold on;
plot(seconds, goalLinSqueezed(3,:));
plot([seconds(1), seconds(end)], [wLinGReal(3), wLinGReal(3)]);
%vertical line to show where insertion begin
plot([secInsertion; secInsertion], [ylim]', '--m');
text([secInsertion+2], [8.465], {'\rightarrow Inside the hole'}, 'Color','magenta', 'FontSize',14);
hold off;
xlab3 = xlabel('time [s]');
ylab3 = ylabel('Vectors (Z) [m]');
tq3 = title("Z Position of tool, goal and goalReal");
leg3 = legend('$z_{tool}$', '$z_{goal}$', '$z_{goalReal}$');

%% plot x ang
subplot(2, 3, 4);
plot(seconds, toolAng(1,:));
hold on
plot(seconds, goalAng(1,:));
plot([seconds(1), seconds(end)], [wAngGReal(1), wAngGReal(1)]);
%vertical line to show where insertion begin
plot([secInsertion; secInsertion], [ylim]', '--m');
text([secInsertion+2], [0.0003], {'\rightarrow Inside the hole'}, 'Color','magenta', 'FontSize',14);
hold off;
xlab4 = xlabel('time [s]');
ylab4 = ylabel('Vectors (roll) [rad]');
tq4 = title("Roll of tool, goal and goalReal");
leg4 = legend('$roll_{tool}$', '$roll_{goal}$', '$roll_{goalReal}$');

%% plot y ang
subplot(2, 3, 5);
plot(seconds, toolAng(2,:));
hold on
plot(seconds, goalAng(2,:));
plot([seconds(1), seconds(end)], [wAngGReal(2), wAngGReal(2)]);
%vertical line to show where insertion begin
plot([secInsertion; secInsertion], [ylim]', '--m');
text([secInsertion+2], [0.00012], {'\rightarrow Inside the hole'}, 'Color','magenta', 'FontSize',14);
hold off;
xlab5 = xlabel('time [s]');
ylab5 = ylabel('Vectors (pitch) [rad]');
tq5 = title("Pitch of tool, goal and goalReal");
leg5 = legend('$pitch_{tool}$', '$pitch_{goal}$', '$pitch_{goalReal}$');

%% plot z ang
subplot(2, 3, 6);
plot(seconds, toolAng(3,:));
hold on
plot(seconds, goalAng(3,:));
plot([seconds(1), seconds(end)], [wAngGReal(3), wAngGReal(3)]);
%vertical line to show where insertion begin
plot([secInsertion; secInsertion], [ylim]', '--m');
text(secInsertion+2, -1.4475, '\rightarrow Inside the hole', 'Color','magenta', 'FontSize',14);
hold off;
ylim([-1.48, -1.44]);
xlab6 = xlabel('time [s]');
ylab6 = ylabel('Vectors (yaw) [rad]');
tq6 = title("Yaw of tool, goal and goalReal");
leg6 = legend('$yaw_{tool}$', '$yaw_{goal}$', '$yaw_{goalReal}$');

set(ylab1, 'Interpreter', 'latex', 'FontSize' , ylabFontSize);
set (tq1, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set(ylab2, 'Interpreter', 'latex', 'FontSize' , ylabFontSize);
set (tq2, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set(ylab3, 'Interpreter', 'latex', 'FontSize' , ylabFontSize);
set (tq3, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set(ylab4, 'Interpreter', 'latex', 'FontSize' , ylabFontSize);
set (tq4, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set(ylab5, 'Interpreter', 'latex', 'FontSize' , ylabFontSize);
set (tq5, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set(ylab6, 'Interpreter', 'latex', 'FontSize' , ylabFontSize);
set (tq6, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set(leg1, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set(leg2, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set(leg3, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set(leg4, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set(leg5, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set(leg6, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set(xlab1, 'Interpreter', 'latex', 'FontSize' , xlabFontSize);
set(xlab2, 'Interpreter', 'latex', 'FontSize' , xlabFontSize);
set(xlab3, 'Interpreter', 'latex', 'FontSize' , xlabFontSize);
set(xlab4, 'Interpreter', 'latex', 'FontSize' , xlabFontSize);
set(xlab5, 'Interpreter', 'latex', 'FontSize' , xlabFontSize);
set(xlab6, 'Interpreter', 'latex', 'FontSize' , xlabFontSize);






%% old plot ang all togheter
% subplot(2,3,[4,5,6]);
% plot(seconds, toolAng);
% hold on
% plot(seconds, goalAng);
% hold off;
% xlab = xlabel('time [s]');
% ylab2 = ylabel('tool and goal angular position  [rad]');
% tq2 = title("Angular position of tool and goal");
% leg2 = legend('$roll_tool$','$pitch_tool$', '$yaw_tool$', '$roll_goal$', '$pitch_goal$', '$yaw_goal$');
% 
% 
% set(ylab1, 'Interpreter', 'none', 'FontSize' , ylabFontSize);
% set (tq1, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
% set(ylab2, 'Interpreter', 'none', 'FontSize' , ylabFontSize);
% set (tq2, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
% set(leg1, 'Interpreter', 'latex', 'FontSize' , legFontSize);
% set(leg2, 'Interpreter', 'latex', 'FontSize' , legFontSize);
% 
% hold off
% 
% 
