function plotStressTool(rootPath, coordName)

legFontSize = 13;
titleFontSize = 16;
ylabFontSize = 15;

fileName = "stressTool.txt";
errors = importMatrices(strcat(rootPath, coordName, fileName));
nRow = size(errors, 1);
nStep = size(errors, 3);

global sControlLoop
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

figure
hold on;
for i = 1:nRow
    a(:) = errors(i,1,:);
    plot(seconds, a);
end
xlabel('time [s]');
ylab = ylabel('errors [m, rad]');

tq = title("Cartesian Error between the two coincident pipes");

leg = legend('$x$','$y$', '$z$', '$roll$', '$pitch$', '$yaw$');
set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set(ylab, 'Interpreter', 'none', 'FontSize' , ylabFontSize);

set (tq, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
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
