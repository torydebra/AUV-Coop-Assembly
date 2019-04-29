function plotDifferenceCoopVel(rootPath, robotNameA, robotNameB)

legFontSize = 13;
titleFontSize = 16;
ylabFontSize = 15;

xDotstr = '/toolCartVelCoop.txt';
xDotA = importMatrices(strcat(rootPath, robotNameA, xDotstr));
xDotB = importMatrices(strcat(rootPath, robotNameB, xDotstr));
nRow = min(size(xDotA, 1), size(xDotB, 1));
nStep = min(size(xDotA, 3), size(xDotB, 3));

%millisecond indicated in missionManager
global sControlLoop
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

diff = xDotA - xDotB;

% plot joint commands
figure
hold on;
for i = 1:nRow
    a(:) = diff(i,1,:);
    plot(seconds, a);
end
xlabel('time [s]');
ylab = ylabel('differences');
leg = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$', '$w_x$', '$w_y$', '$w_z$');
tq = title("Difference of cartesian velocity applied to tool");

set(ylab, 'Interpreter', 'none', 'FontSize' , ylabFontSize);
set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (tq, 'Interpreter', 'latex', 'FontSize' , titleFontSize);


hold off