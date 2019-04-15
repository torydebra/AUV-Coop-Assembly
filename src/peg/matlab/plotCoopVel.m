function plotCoopVel(rootPath, coordName)

legFontSize = 13;
titleFontSize = 16;
ylabFontSize = 15;

fileName = strcat('coopVel.txt');
xDot = importMatrices(strcat(rootPath, coordName, fileName));
nRow = size(xDot, 1);
nStep = size(xDot, 3);

%millisecond indicated in missionManager
sControlLoop = 0.1;
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

%plot vel
figure
hold on;
for i = 1:nRow
    a(:) = xDot(i,1,:);
    plot(seconds, a);
end
xlabel('time [s]');
ylab = ylabel(strcat('Cooperative vel [m/s], [rad/s]'));
tq = title('Cooperative velocity of tool calculated');

if nRow == 6
    leg = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$', '$w_x$', '$w_y$', '$w_z$');
elseif nRow == 5
    leg = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$', '$w_y$', '$w_z$');
    
end
set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (tq, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set (ylab, 'Interpreter', 'none', 'FontSize', ylabFontSize);
hold off