function plotVector(rootPath, robotName, vecName)

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
hold on;
for i = 1:nRow
    a(:) = vec(i,1,:);
    plot(seconds, a);
end
xlabel('time [s]');
leg = legend('x','y', 'z');
ylab = ylabel(' vector [?]');
tq = title(strcat(robotName, " "  ,vecName));
set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (tq, 'Interpreter', 'none', 'FontSize' , titleFontSize);
set (ylab, 'Interpreter', 'none', 'FontSize', ylabFontSize);
hold off
