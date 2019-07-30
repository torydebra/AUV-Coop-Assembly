function plotForcesTorquesWorld(rootPath, robotName, strNorm)

legFontSize = 19;
titleFontSize = 18;
ylabFontSize = 17;
xlabFontSize = 17;

coordName = 'Coordinator/';
vecFor = importMatrices(strcat(rootPath, robotName, '/forces.txt'));
vecTor = importMatrices(strcat(rootPath, robotName, '/torques.txt'));
fileName = "wTt.txt";
wTt = importMatrices(strcat(rootPath, coordName, fileName));
nStep = min( size(vecFor, 3), size(wTt, 3));

%project in world
w_vecFor =zeros(3,nStep);
w_vecTor =zeros(3,nStep);
for i=1:nStep
  w_vecFor (:,i) = wTt(1:3,1:3,i) * vecFor(:,:,i);
  w_vecTor (:,i) = wTt(1:3,1:3,i) * vecTor(:,:,i);
end

%millisecond indicated in missionManager
global sControlLoop
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

global secInsertion

% plot joint commands
figure('Renderer', 'painters', 'Position', [0 0 710 550])
subplot(2,1,1)
hold on;
if strcmp(strNorm, 'yes')
  plot(seconds, vecnorm(w_vecFor));
  ylab = ylabel('norm of force [N]');
  tq = title("Norm of force vector projected in world");
  %xlim([0,250]);
  %ylim([0,15]);
else
  plot(seconds, w_vecFor);
  leg = legend('x','y', 'z');
  ylab = ylabel('forces [N]');
  tq = title(strcat("forces projected in world"));
  set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
  %xlim([0,170]);
end
xlab = xlabel('time [s]');
plot([secInsertion; secInsertion], [ylim]', '--m');
text([secInsertion+2], [1.85], {'\rightarrow Inside the hole'}, 'Color', 'magenta', 'FontSize',14);
hold off



subplot(2,1,2)
hold on;
if strcmp(strNorm, 'yes')
  plot(seconds, vecnorm(w_vecTor));
  ylab2 = ylabel('norm of torque [N*m]');
  tq2 = title('Norm of torque vector projected in world');
    %xlim([0,250]);
      %ylim([0,20]);
      
else
  plot(seconds, w_vecTor);
  leg2 = legend('x','y', 'z');
  ylab2 = ylabel('torques [N*m]');
  tq2 = title(strcat("torques projected in world"));
  set(leg2, 'Interpreter', 'latex', 'FontSize' , legFontSize);
    %xlim([0,170]);

end
xlab2 = xlabel('time [s]');

plot([secInsertion; secInsertion], [ylim]', '--m');
text([secInsertion+2], [0.28], {'\rightarrow Inside the hole'}, 'Color', 'magenta', 'FontSize',14);
hold off

set (tq, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set (ylab, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
set (xlab, 'Interpreter', 'latex', 'FontSize', xlabFontSize);
set (tq2, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set (ylab2, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
set (xlab2, 'Interpreter', 'latex', 'FontSize', xlabFontSize);



