clearvars;

rootPath = '~/logPegBella/all/';
robotNameA = 'g500_A';
robotNameB = 'g500_B';
coordName = 'Coordinator/';

%millisecond indicated in missionManager
global sControlLoop
sControlLoop = 0.1;
%sControlLoop = 0.05; %original 0.1 = 100 ms

%% plot Task things

%  taskName = 'FORCE_INSERTION';
%taskName = 'JOINT_LIMIT';
% 
%plotTask(rootPath, robotNameA, taskName);
%plotTask(rootPath, robotNameB, taskName);


%% plot yDots (command sent to robot)
%plotYDot(rootPath, robotNameA, 'yDotTPIK1');
%plotYDot(rootPath, robotNameA, 'yDotFinal');
%plotYDotDivided(rootPath, robotNameA, 'yDotTPIK1'); %ang e lin vel of vehicle divided
% plotYDotDivided(rootPath, robotNameA, 'yDotFinal');
% plotYDotDivided(rootPath, robotNameA, 'yDotFinalWithCollision');
plotForcesWorld(rootPath, robotNameA);
plotTorquesWorld(rootPath, robotNameA);
%plot6DVectorDivided(rootPath, robotNameA, 'toolVel4Collision');
%plotVector(rootPath, robotNameA, 'forces');
%plotVector(rootPath, robotNameA, 'torques');


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
%plotDifferenceCoopVel(rootPath, robotNameA, robotNameB)
%plotStressTool(rootPath, coordName);


%% diff tra feasabile e non coop velocities
% legFontSize = 13;
% titleFontSize = 16;
% ylabFontSize = 15;
% 
% fileName = strcat('nonCoopVel', robotNameB, '.txt');
% xDot1 = importMatrices(strcat(rootPath, coordName, fileName));
% nRow = size(xDot1, 1);
% nStep = size(xDot1, 3);
% 
% %millisecond indicated in missionManager
% totSecondPassed = sControlLoop*(nStep-1);
% seconds = 0:sControlLoop:totSecondPassed;
% 
% fileName = strcat('idealTool', '.txt');
% xDot2 = importMatrices(strcat(rootPath, coordName, fileName));
% 
% diff = xDot2 - xDot1;
% figure
% hold on;
% for i = 1:nRow
%     a(:) = diff(i,1,:);
%     plot(seconds, a);
% end
% xlabel('time [s]');

%% Plot goal moving for change_goal and pose tool
plotTransformMatrices(rootPath, coordName);
plotGenericErrorDivided(strcat(rootPath, coordName, "realgoal_Tool_error.txt"));


%% Vision
% pathName ="logVisionGood/templ/mono/g500_C/errorMonoL.txt";
% %plotGenericErrorDividedNorm(pathName)
% pathName ="logVisionGood/templ/mono/g500_C/errorMonoR.txt";
% %plotGenericErrorDividedNorm(pathName)
% 
% 
% pathName ="logVisionGood/templ/stereo/g500_C/errorStereoL.txt";
% plotGenericErrorDividedNorm(pathName)
% pathName ="logVisionGood/templ/depth/g500_C/errorStereoL.txt";
% plotGenericErrorDividedNorm(pathName)