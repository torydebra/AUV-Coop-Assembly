rootPath = '~/logPeg15/';
robotNameA = 'g500_A';
robotNameB = 'g500_B';
ydotName = 'toolCartVelCoop';

legFontSize = 13;
titleFontSize = 16;
ylabFontSize = 15;

xDotstr = strcat( '/', ydotName, '.txt');
xDotA = importMatrices(strcat(rootPath, robotNameA, xDotstr));
xDotB = importMatrices(strcat(rootPath, robotNameB, xDotstr));
nRow = min(size(xDotA, 1), size(xDotB, 1));
nStep = min(size(xDotA, 3), size(xDotB, 3));

%millisecond indicated in missionManager
sControlLoop = 0.1;
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

diff = xDotA - xDotB;

% plot joint commands
figure
hold on;
for i = 1:6
    a(:) = diff(i,1,:);
    plot(seconds, a);
end
xlabel('time [s]');
leg = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$', '$w_x$', '$w_y$', '$w_z$');
set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
hold off
