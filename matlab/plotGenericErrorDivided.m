function plotGenericErrorDivided(pathName, fileName, strNorm)

legFontSize = 19;
titleFontSize = 18;
ylabFontSize = 17;
xlabFontSize = 17;


errors = importMatrices(pathName);
errorsSqueezed = squeeze(errors);
nStep = size(errors, 3);

global sControlLoop
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

figure('Renderer', 'painters', 'Position', [0 0 710 550])
subplot(2,1,1);
hold on;
if strcmp(strNorm,'yes')
      plot(seconds, vecnorm(errorsSqueezed(1:3,:))  );
else
      plot(seconds, errorsSqueezed(1:3,:));
end
hold off;
xlab1 = xlabel('time [s]');

if strcmp(fileName,'realgoal_Tool_error')
    if strcmp(strNorm,'yes')
        ylab1 = ylabel('norm of error [m]');
        tq1 = title("Norm of Linear error between goal (real) and tool");
    else
        ylab1 = ylabel('errors [m]');
        tq1 = title("Linear error between goal (real) and tool");
        leg1 = legend('$x$','$y$', '$z$');
        set(leg1, 'Interpreter', 'latex', 'FontSize' , legFontSize);
    end
else
  ylab1 = ylabel('errors [m]');
  tq1 = title("Cartesian Linear Error");
  leg1 = legend('$x$','$y$', '$z$');
  set(leg1, 'Interpreter', 'latex', 'FontSize' , legFontSize);
end


subplot(2,1,2);
hold on;
if strcmp(strNorm,'yes')
  plot(seconds, vecnorm(errorsSqueezed(4:6,:)));
else
  plot(seconds, errorsSqueezed(4:6,:) );
end
hold off;
xlab2 = xlabel('time [s]');

if strcmp(fileName,'realgoal_Tool_error')
    if strcmp(strNorm,'yes')
        ylab2 = ylabel('norm of error [rad]');
        tq2 = title("Norm of Angular error between goal (real) and tool");
    else
        ylab2 = ylabel('errors [rad]');
        tq2 = title("Angular error between goal (real) and tool");
        leg2 = legend('$roll$', '$pitch$', '$yaw$');
        set(leg2, 'Interpreter', 'latex', 'FontSize' , legFontSize);
    end
else
  ylab2 = ylabel('errors [rad]');
  tq2 = title("Cartesian Angular Error");
  leg2 = legend('$roll$', '$pitch$', '$yaw$');
  set(leg2, 'Interpreter', 'latex', 'FontSize' , legFontSize);
end


set(ylab1, 'Interpreter', 'latex', 'FontSize' , ylabFontSize);
set (tq1, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set(ylab2, 'Interpreter', 'latex', 'FontSize' , ylabFontSize);
set (tq2, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set (xlab1, 'Interpreter', 'latex', 'FontSize', xlabFontSize);
set (xlab2, 'Interpreter', 'latex', 'FontSize', xlabFontSize);






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
