function plotTorquesWorld(rootPath, robotName, strNorm)

legFontSize = 13;
titleFontSize = 16;
ylabFontSize = 15;
xlabFontSize = 16;

coordName = 'Coordinator/';
vec = importMatrices(strcat(rootPath, robotName, '/torques.txt'));
fileName = "wTt.txt";
wTt = importMatrices(strcat(rootPath, coordName, fileName));
nStep = min( size(vec, 3), size(wTt, 3));

%project in world
w_vec =zeros(3,nStep);
for i=1:nStep
  w_vec (:,i) = wTt(1:3,1:3,i) * vec(:,:,i);
end

%millisecond indicated in missionManager
global sControlLoop
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

% plot joint commands
figure
if strcmp(strNorm, 'yes')
  plot(seconds, vecnorm(w_vec));
  ylab = ylabel('norm of torque [N*m]');
  tq = title('Norm of torque vector projected in world');
else
  plot(seconds, w_vec);
  leg = legend('x','y', 'z');
  ylab = ylabel('torques [N*m]');
  tq = title(strcat("torques projected in world"));
  set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);

end
xlab = xlabel('time [s]');

set (tq, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set (ylab, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
set (xlab, 'Interpreter', 'latex', 'FontSize', xlabFontSize);

