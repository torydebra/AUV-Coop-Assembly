function plotTorquesWorld(rootPath, robotName)

legFontSize = 13;
titleFontSize = 16;
ylabFontSize = 15;
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
hold on;
plot(seconds, w_vec);
xlabel('time [s]');
leg = legend('x','y', 'z');
ylab = ylabel(' vector [N*m]');
tq = title(strcat(robotName, " torques projected in world"));
set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (tq, 'Interpreter', 'none', 'FontSize' , titleFontSize);
set (ylab, 'Interpreter', 'none', 'FontSize', ylabFontSize);
hold off
