function plotNonCoopVel(rootPath, coordName, robotName)

legFontSize = 13;
titleFontSize = 16;
ylabFontSize = 15;

fileName = strcat('nonCoopVel', robotName, '.txt');
xDot = importMatrices(strcat(rootPath, coordName, fileName));
nRow = size(xDot, 1);
nStep = size(xDot, 3);

%millisecond indicated in missionManager
global sControlLoop
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
ylab = ylabel(strcat('NonCooperative vel of  ' , robotName ,' [m/s], [rad/s]'));
tq = title(strcat(robotName, ' Non Cooperative vel of tool $J_{tool} \dot{y}$'));

if nRow == 6
    leg = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$', '$w_x$', '$w_y$', '$w_z$');
elseif nRow == 5
    leg = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$', '$w_y$', '$w_z$');
    
end
set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (tq, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set (ylab, 'Interpreter', 'none', 'FontSize', ylabFontSize);
hold off
