function plotTask(rootPath, robotName, taskName)

filename = strcat(rootPath, robotName, "/", taskName);
legFontSize = 22;
titleFontSize = 18;
ylabFontSize = 19;
xlabFontSize = 19;


%% plot refs (vector n*1)
refs = importMatrices(strcat(filename, '/reference.txt'));
nRow = size(refs, 1);
nStep = size(refs, 3);

%millisecond indicated in missionManager
global sControlLoop;
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

figure('Renderer', 'painters', 'Position', [0 0 650 500])
hold on;
for i = 1:nRow
    a(:) = refs(i,1,:);
    plot(seconds, a);
end
xlab = xlabel('time [s]');

if  strcmp(taskName, 'PIPE_REACHING_GOAL') 
    leg = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$', '$w_x$', '$w_y$', '$w_z$');
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
elseif strcmp(taskName, 'FORCE_INSERTION')
    if nRow == 1 %only force norm
       leg = legend('$\dot{\bar{x}}$');
       ylab = ylabel('reference (norm)');
       
    elseif nRow == 2 %norm of force and torque. BEST METHOD
    	 leg = legend('$\dot{\bar{x}}_f$', '$\dot{\bar{x}}_m$');
       ylab = ylabel('reference');
       if strcmp(robotName, 'g500_A')
       	 tr = title('Robot A: Force-Torque Task Reference');
       else
        tr = title('Robot B: Force-Torque Task Reference');
       end
       
    elseif nRow == 3 % only force
        leg = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$');
        ylab = ylabel('reference []');
    end

end

if ~strcmp(taskName, 'FORCE_INSERTION') 
  tr = title(strcat(robotName, " ", taskName));
end
set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize );
set (tr, 'Interpreter', 'latex', 'FontSize', titleFontSize);
set (ylab, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
set (xlab, 'Interpreter', 'latex', 'FontSize', xlabFontSize);


%% plot errors
% error = importMatrices(strcat(filename, '/error.txt'));
% nRow = size(refs, 1);
% nStep = size(refs, 3);
% 
% %millisecond indicated in missionManager
% totSecondPassed = sControlLoop*(nStep-1);
% seconds = 0:sControlLoop:totSecondPassed;
% 
% figure
% hold on;
% for i = 1:nRow
%     a(:) = error(i,1,:);
%     plot(seconds, a);
% end
% xlab = xlabel('time [s]');
% 
% if  strcmp(taskName, 'PIPE_REACHING_GOAL') 
%     leg = legend('$x_{err}$','$y_{err}$', '$z_{err}$', '$pitch_{err}$', '$yaw_{err}$');
%     ylab = ylabel('error [m],[rad]');
% elseif  strcmp(taskName, 'JOINT_LIMIT') 
%     leg = legend('$q1_{err}$','$q2_{err}$', '$q3_{err}$', '$q4_{err}$');
%     ylab = ylabel('error [rad]');
% elseif  strcmp(taskName, 'ARM_SHAPE')
%      if nRow == 1 % scalar type (norm)
%         leg = legend('$norm_{err}$');
%         ylab = ylabel('error (norm)');
%     else
%         leg = legend('$q1_{err}$','$q2_{err}$', '$q3_{err}$', '$q4_{err}$');
%         ylab = ylabel('errors [rad]');
%      end
%  elseif strcmp(taskName, 'FORCE_INSERTION')
%      if nRow == 1 %only force norm
%         leg = legend('$norm_{err}$');
%         ylab = ylabel('error (norm)');     
%         
%     elseif nRow == 2 %norm of force and torque. BEST METHOD
%     	 leg = legend('$err_f$', '$err_m$');
%        ylab = ylabel('error (norm)');
%        
%      elseif nRow == 3 % only force
%         leg = legend('$x_{err}$','$y_{err}$', '$z_{err}$');
%         ylab = ylabel('error [m]');
%     end
% end
% 
% te = title(strcat(robotName, " ", taskName));
% set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
% set (te, 'Interpreter', 'latex', 'FontSize', titleFontSize);
% set (ylab, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
% 
% hold off;


%% plot activations
act = importMatrices(strcat(filename, '/activation.txt'));
nRow = size(refs, 1);
nStep = size(refs, 3);

%millisecond indicated in missionManager
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

figure('Renderer', 'painters', 'Position', [0 0 650 500])
hold on;
for i = 1:nRow
    a(:) = act(i,1,:);
    plot(seconds, a);
end
xlab = xlabel('time [s]');

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
elseif strcmp(taskName, 'FORCE_INSERTION')
    if nRow == 1 % only norm force
        leg = legend('$A$');
        ylab = ylabel('activation');
        error (norm)
        
  elseif nRow == 2 %norm of force and torque. BEST METHOD
    	 leg = legend('$a_f$', '$a_m$');
       ylab = ylabel('activations');
       if strcmp(robotName, 'g500_A')
       	 ta = title('Robot A: Force-Torque Task Activation');
       else
         ta = title('Robot B: Force-Torque Task Activation');
       end
       
	elseif nRow == 3 % only force
        leg = legend('$A_{11}$','$A_{22}$', '$A_{33}$');
        ylab = ylabel('activations');
    end
end

if ~strcmp(taskName, 'FORCE_INSERTION') 
  ta = title(strcat(robotName, " " ,taskName));
end
set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
set (ta, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set (ylab, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
set (xlab, 'Interpreter', 'latex', 'FontSize', xlabFontSize);


%% plot jacob-1*ref (for ydot generated by the single task) only for force insertion for now
if  strcmp(taskName, 'FORCE_INSERTION')   
  jacobs = importMatrices(strcat(rootPath, robotName, '/forceJacob.txt'));
  yDotSingle = zeros(10,nStep);
  figure

  for i=1:nStep
    yDotSingle(:,i) = pinv(jacobs(:,:,i)) * refs(:,:,i);  
  end
  plot (seconds, yDotSingle)
  legForcJac = legend('$\dot{q}_1$','$\dot{q}_2$', '$\dot{q}_3$', '$\dot{q}_4$', ...
                      '$\dot{q}_5$','$\dot{q}_6$', '$\dot{q}_7$', ...
                      '$\dot{q}_9$', '$\dot{q}_8$','$\dot{q}_10$');
   set(legForcJac, 'Interpreter', 'latex', 'FontSize' , legFontSize);
     

end


