function plotGenericErrorDivided(pathName)

legFontSize = 13;
titleFontSize = 13;
ylabFontSize = 12;

errors = importMatrices(pathName);
nRow = size(errors, 1);
nStep = size(errors, 3);

global sControlLoop
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

figure
subplot(2,1,1);
hold on;
for i = 1:3
    a(:) = errors(i,1,:);
    plot(seconds, a);
end
hold off;
xlabel('time [s]');
ylab1 = ylabel('errors [m]');
tq1 = title("Cartesian Linear Error");
leg1 = legend('$x$','$y$', '$z$');


subplot(2,1,2);
hold on;
for i = 4:6
    a(:) = errors(i,1,:);
    plot(seconds, a);
end
hold off;
xlabel('time [s]');
ylab2 = ylabel('errors [rad]');
tq2 = title("Cartesian Angular Error");
leg2 = legend('$roll$', '$pitch$', '$yaw$');


set(leg1, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set(leg2, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set(ylab1, 'Interpreter', 'none', 'FontSize' , ylabFontSize);
set (tq1, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set(ylab2, 'Interpreter', 'none', 'FontSize' , ylabFontSize);
set (tq2, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
hold off






%% plot vel norm
% figure
% hold on;
% for i = 1:nRow
%     a(:) = norms(i,1,:);
%     plot(seconds, a);
% end
% xlabel('time [s]');
% ylab = ylabel('norms');
% 
% tq = title("Norms (lin, ang) of Cartesian Error between the two coincident pipes");
% 
% leg = legend('Linear', 'Angular');
% set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
% set(ylab, 'Interpreter', 'none', 'FontSize' , ylabFontSize);
% 
% set (tq, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
% hold off
