
clearvars;
close all;

%% plot refs (vector n*1)
refs = importMatrices('~/logPeg/g500_B/PIPE_REACHING_GOAL/reference.txt');

nRow = size(refs, 1);
nStep = size(refs, 3);

%millisecond indicated in missionManager
sControlLoop = 0.1;
totSecondPassed = sControlLoop*(nStep-1);

seconds = 0:sControlLoop:totSecondPassed;

hold on;
for i = 1:nRow
    a(:) = refs(i,1,:);
    plot(seconds, a);
end
hold off;
legend('xdot','ydot', 'zdot', 'w_y', 'w_z');
xlabel('seconds (s)')
ylabel('references')