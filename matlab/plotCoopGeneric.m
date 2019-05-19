function plotCoopGeneric(rootPath, coordName, file)

legFontSize = 13;
titleFontSize = 16;
ylabFontSize = 15;

fileName = strcat(file, '.txt');
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

tq = title(file);

if nRow == 6
    leg = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$', '$w_x$', '$w_y$', '$w_z$');
    set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
elseif nRow == 5
    leg = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$', '$w_y$', '$w_z$');
    set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
    
end

set (tq, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
hold off
