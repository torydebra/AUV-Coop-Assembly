clearvars;

rootPath = 'log/logPegBella/withVision/';
robotNameA = 'g500_A';
robotNameB = 'g500_B';
coordName = 'Coordinator/';
set(groot,'defaultLineLineWidth',1.5)

%millisecond indicated in missionManager
global sControlLoop
sControlLoop = 0.1;

%second of insertion phase
global secInsertion
secInsertion = 77; %withVision exp
%secInsertion = 23.4; %errorALL exp

%% plot yDot
%plotYDotDivided(rootPath, robotNameA, 'yDotFinalWithCollision');

%% plot Force Task things

taskName = 'FORCE_INSERTION';
plotTask(rootPath, robotNameA, taskName);
plotTask(rootPath, robotNameB, taskName);

%% plot yDots and related thing
%plotYDotDivided(rootPath, robotNameA, 'yDotTPIK1'); 
%plotYDotDivided(rootPath, robotNameB, 'yDotTPIK1'); 
plotYDotDivided(rootPath, robotNameA, 'yDotFinal'); %after cooperation
plotYDotDivided(rootPath, robotNameB, 'yDotFinal'); %after cooperation
%plotYDotDivided(rootPath, robotNameA, 'yDotFinalWithCollision'); %after cooperation with collision


plotForcesTorquesWorld(rootPath, robotNameA, 'yes');  %yes to plot norm
%plotForcesTorquesWorld(rootPath, robotNameB, 'yes');  %yes to plot norm


plot6DVectorDivided(rootPath, robotNameA, 'toolVel4Collision');
plot6DVectorDivided(rootPath, robotNameB, 'toolVel4Grasp');


%% plot coord things 
%here are plotted velocities of tool, not the final one but the
%intermidiate that are input and output of the cooperation policy
%plotNonCoopVel(rootPath, coordName, robotNameA);
%plotNonCoopVel(rootPath, coordName, robotNameB);
plotCoopVel(rootPath, coordName); %this is the feasible one


%% Plot goal moving for change_goal and pose tool
plotTransformMatrices(rootPath, coordName);
plotGenericErrorDivided(strcat(rootPath, coordName, "realgoal_Tool_error.txt"),'realgoal_Tool_error', 'yes');


%% Vision
%pathName = rootPath + "g500_C/errorStereoL.txt";
%plotGenericErrorDivided(pathName, "errorStereo", 'no')
