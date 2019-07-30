refsA = importMatrices('~/logPegBella/withVision/15/g500_A/forces.txt');
refsB = importMatrices('~/logPegBella/withVision/15/g500_B/forces.txt');
nStep = size(refsB, 3);
%diffRefs = refsB - refsA;

%diffRefs = squeeze(diffRefs);

global sControlLoop;
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

%figure('Renderer', 'painters', 'Position', [0 0 650 500])
%plot(seconds, diffRefs);
%xlab = xlabel('time [s]');

figure
plot(seconds, squeeze(refsA));

figure
plot(seconds, squeeze(refsB));

