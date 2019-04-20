function plotStressTool(rootPath, coordName)

legFontSize = 13;
titleFontSize = 16;
ylabFontSize = 15;

fileName = "stressTool.txt";
norms = importMatrices(strcat(rootPath, coordName, fileName));
nRow = size(norms, 1);
nStep = size(norms, 3);

%millisecond indicated in missionManager
sControlLoop = 0.1;
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

%plot vel
figure
hold on;
for i = 1:nRow
    a(:) = norms(i,1,:);
    plot(seconds, a);
end
xlabel('time [s]');
ylab = ylabel('norms');

tq = title("Norms (lin, ang) of Cartesian Error between the two coincident pipes");

leg = legend('Linear', 'Angular');
set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set(ylab, 'Interpreter', 'none', 'FontSize' , ylabFontSize);

set (tq, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
hold off
