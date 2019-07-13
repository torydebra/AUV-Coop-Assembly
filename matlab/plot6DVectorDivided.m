function plot6DVectorDivided(rootPath, robotName, vecName)

legFontSize = 13;
titleFontSize = 16;
ylabFontSize = 15;

vecNamestr = strcat( '/', vecName, '.txt');
vec = importMatrices(strcat(rootPath, robotName, vecNamestr));
nRow = size(vec, 1);
nStep = size(vec, 3);

%millisecond indicated in missionManager
global sControlLoop
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

% plot joint commands
figure
subplot(2,1,1);
hold on;
for i = 1:3
    a(:) = vec(i,1,:);
    plot(seconds, a);
end
xlabel('time [s]');
leg = legend('x','y', 'z');
ylab = ylabel(' vector [?]');
tq = title(strcat(robotName, " LINEAR "  ,vecName));
set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (tq, 'Interpreter', 'none', 'FontSize' , titleFontSize);
set (ylab, 'Interpreter', 'none', 'FontSize', ylabFontSize);
hold off

subplot(2,1,2);
hold on;
for i = 4:6
    a(:) = vec(i,1,:);
    plot(seconds, a);
end
xlabel('time [s]');
leg2 = legend('x_{ang}','y_{ang}', 'z_{ang}');
ylab2 = ylabel(' vector [?]');
tq2 = title(strcat(robotName, " ANGULAR "  ,vecName));
set(leg2, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (tq2, 'Interpreter', 'none', 'FontSize' , titleFontSize);
set (ylab2, 'Interpreter', 'none', 'FontSize', ylabFontSize);
hold off
