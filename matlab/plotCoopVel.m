function plotCoopVel(rootPath, coordName)

legFontSize = 19;
titleFontSize = 18;
ylabFontSize = 17;
xlabFontSize = 17;

fileName = strcat('coopVel.txt');
xDot = importMatrices(strcat(rootPath, coordName, fileName));
xDotSqueez = squeeze(xDot);
nRow = size(xDot, 1);
nStep = size(xDot, 3);

%millisecond indicated in missionManager
global sControlLoop
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

figure('Renderer', 'painters', 'Position', [0 0 710 550])
subplot(2,1,1)
plot(seconds, xDotSqueez(1:3,:));

xlab = xlabel('time [s]');
ylab = ylabel('Velocities [m/s]');
tq = title('Cooperative Tool Linear Velocities');
leg = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$'); 

subplot(2,1,2)
plot(seconds, xDotSqueez(4:6,:));
xlab2 = xlabel('time [s]');
ylab2 = ylabel('Velocities [rad/s]');
tq2 = title('Cooperative Tool Angular Velocities');
leg2 = legend('$w_x$', '$w_y$', '$w_z$');

set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (tq, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set (ylab, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
set (xlab, 'Interpreter', 'latex', 'FontSize', xlabFontSize);
set(leg2, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (tq2, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set (ylab2, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
set (xlab2, 'Interpreter', 'latex', 'FontSize', xlabFontSize);





%% plot vel OLD
% figure
% hold on;
% for i = 1:nRow
%     a(:) = xDot(i,1,:);
%     plot(seconds, a);
% end
% xlabel('time [s]');
% ylab = ylabel(strcat('Cooperative vel [m/s], [rad/s]'));
% tq = title('Cooperative velocity of tool');
% 
% if nRow == 6
%     leg = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$', '$w_x$', '$w_y$', '$w_z$');
% elseif nRow == 5
%     leg = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$', '$w_y$', '$w_z$');
%     
% end
% set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
% set (tq, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
% set (ylab, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
% hold off