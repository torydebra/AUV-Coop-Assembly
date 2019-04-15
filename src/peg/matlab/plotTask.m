function plotTask(rootPath, robotName, taskName)

filename = strcat(rootPath, robotName, "/", taskName);
legFontSize = 13;
titleFontSize = 16;
ylabFontSize = 15;

%% plot refs (vector n*1)
refs = importMatrices(strcat(filename, '/reference.txt'));
nRow = size(refs, 1);
nStep = size(refs, 3);

%millisecond indicated in missionManager
sControlLoop = 0.1;
totSecondPassed = sControlLoop*(nStep-1);

seconds = 0:sControlLoop:totSecondPassed;

figure
hold on;
for i = 1:nRow
    a(:) = refs(i,1,:);
    plot(seconds, a);
end
xlabel('time [s]');

if  strcmp(taskName, 'PIPE_REACHING_GOAL') 
    leg = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$', '$w_y$', '$w_z$');
    ylab = ylabel('references [m/s], [rad/s]');
elseif strcmp(taskName, 'JOINT_LIMIT') 
    leg = legend('$\dot{q}_1$','$\dot{q}_2$', '$\dot{q}_3$', '$\dot{q}_4$');
    ylab = ylabel('references [rad/s]');
elseif strcmp(taskName, 'HORIZONTAL_ATTITUDE')
    leg = legend('$\dot{\bar{x}}$');
    ylab = ylabel('references');   
elseif strcmp(taskName, 'ARM_SHAPE')
    if nRow == 1 % scalar type (norm)
        leg = legend('$\dot{\bar{x}}$');
        ylab = ylabel('reference (norm)');
    else
        leg = legend('$\dot{q}_1$','$\dot{q}_2$', '$\dot{q}_3$', '$\dot{q}_4$');
        ylab = ylabel('references [rad/s]');
    end 

end

tr = title(strcat(robotName, taskName));
set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize );
set (tr, 'Interpreter', 'none', 'FontSize', titleFontSize);
set (ylab, 'Interpreter', 'none', 'FontSize', ylabFontSize);
hold off;

%% plot errors
error = importMatrices(strcat(filename, '/error.txt'));
nRow = size(refs, 1);
nStep = size(refs, 3);

%millisecond indicated in missionManager
sControlLoop = 0.1;
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

figure
hold on;
for i = 1:nRow
    a(:) = error(i,1,:);
    plot(seconds, a);
end
xlabel('time [s]');

if  strcmp(taskName, 'PIPE_REACHING_GOAL') 
    leg = legend('$x_{err}$','$y_{err}$', '$z_{err}$', '$pitch_{err}$', '$yaw_{err}$');
    ylab = ylabel('error [m],[rad]');
elseif  strcmp(taskName, 'JOINT_LIMIT') 
    leg = legend('$q1_{err}$','$q2_{err}$', '$q3_{err}$', '$q4_{err}$');
    ylab = ylabel('error [rad]');
elseif  strcmp(taskName, 'ARM_SHAPE')
     if nRow == 1 % scalar type (norm)
        leg = legend('$norm_{err}$');
        ylab = ylabel('error (norm)');
    else
        leg = legend('$q1_{err}$','$q2_{err}$', '$q3_{err}$', '$q4_{err}$');
        ylab = ylabel('errors [rad]');
     end
end

te = title(strcat(robotName, taskName));
set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (te, 'Interpreter', 'none', 'FontSize', titleFontSize);
set (ylab, 'Interpreter', 'none', 'FontSize', ylabFontSize);

hold off;


%% plot activations
act = importMatrices(strcat(filename, '/activation.txt'));
nRow = size(refs, 1);
nStep = size(refs, 3);

%millisecond indicated in missionManager
sControlLoop = 0.1;
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

figure
hold on;
for i = 1:nRow
    a(:) = act(i,1,:);
    plot(seconds, a);
end
xlabel('time [s]');

if  strcmp(taskName, 'JOINT_LIMIT')   
    leg = legend('$A_{11}$','$A_{22}$', '$A_{33}$', '$A_{44}$');
    ylab = ylabel('activations');
elseif strcmp(taskName, 'HORIZONTAL_ATTITUDE')
    leg = legend('$A$');
    ylab = ylabel('activation'); 
elseif strcmp(taskName, 'ARM_SHAPE')
    if nRow == 1 % scalar type (norm)
        leg = legend('$A$');
        ylab = ylabel('activation');
    else
      leg = legend('$A_{11}$','$A_{22}$', '$A_{33}$', '$A_{44}$');
      ylab = ylabel('activations');
    end
end

ta = title(strcat(robotName, taskName));
set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (ta, 'Interpreter', 'none', 'FontSize' , titleFontSize);
set (ylab, 'Interpreter', 'none', 'FontSize', ylabFontSize);

hold off;


