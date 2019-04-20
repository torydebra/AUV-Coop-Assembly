clearvars;

rootPath = '~/logStress3/';
robotNameA = 'g500_A';
robotNameB = 'g500_B';
coordName = 'Coordinator/';

%% plot Task things

% taskName = 'ARM_SHAPE';
% 
% plotTask(rootPath, robotName, taskName);

%% plot yDots (command sent to robot)
% plotYDot(rootPath, robotNameA, 'yDotTPIK1');
% plotYDot(rootPath, robotNameA, 'yDotFinal');
% plotYDot(rootPath, robotNameB, 'yDotTPIK1');
% plotYDot(rootPath, robotNameB, 'yDotFinal');

%% plot coord things
% plotNonCoopVel(rootPath, coordName, robotNameA);
% plotNonCoopVel(rootPath, coordName, robotNameB);
% plotCoopVel(rootPath, coordName);
% plotCoopGeneric(rootPath, coordName, 'weightA');
% plotCoopGeneric(rootPath, coordName, 'weightB');
% plotCoopGeneric(rootPath, coordName, 'notFeasibleCoopVel');
% plotCoopGeneric(rootPath, coordName, 'idealTool');
plotStressTool(rootPath, coordName);


