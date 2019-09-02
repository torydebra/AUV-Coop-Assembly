function plotTask(rootPath, robotName, taskName)

filename = strcat(rootPath, robotName, "/", taskName);
legFontSize = 22;
titleFontSize = 18;
ylabFontSize = 19;
xlabFontSize = 19;



refs = importMatrices(strcat(filename, '/reference.txt'));
act = importMatrices(strcat(filename, '/activation.txt'));

nRow = size(refs, 1);
nStep = size(refs, 3);

%millisecond indicated in missionManager
global sControlLoop;
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

global secInsertion

%% plot refs (vector n*1)
% figure('Renderer', 'painters', 'Position', [0 0 650 500])
% hold on;
% for i = 1:nRow
%     a(:) = refs(i,1,:);
%     plot(seconds, a);
%     xlim([secInsertion;180]);
% end
% xlab = xlabel('time [s]');
% 
% if  strcmp(taskName, 'PIPE_REACHING_GOAL') 
%     leg = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$', '$w_x$', '$w_y$', '$w_z$');
%     ylab = ylabel('references [m/s], [rad/s]');
% elseif strcmp(taskName, 'JOINT_LIMIT') 
%     leg = legend('$\dot{q}_1$','$\dot{q}_2$', '$\dot{q}_3$', '$\dot{q}_4$');
%     ylab = ylabel('references [rad/s]');
% elseif strcmp(taskName, 'HORIZONTAL_ATTITUDE')
%     leg = legend('$\dot{\bar{x}}$');
%     ylab = ylabel('references');   
% elseif strcmp(taskName, 'ARM_SHAPE')
%     if nRow == 1 % scalar type (norm)
%         leg = legend('$\dot{\bar{x}}$');
%         ylab = ylabel('reference (norm)');
%     else
%         leg = legend('$\dot{q}_1$','$\dot{q}_2$', '$\dot{q}_3$', '$\dot{q}_4$');
%         ylab = ylabel('references [rad/s]');
%     end 
% elseif strcmp(taskName, 'FORCE_INSERTION')
%     if nRow == 1 %only force norm
%        leg = legend('$\dot{\bar{x}}$');
%        ylab = ylabel('reference (norm)');
%        
%     elseif nRow == 2 %norm of force and torque. BEST METHOD
%     	 leg = legend('$\dot{\bar{x}}_f$', '$\dot{\bar{x}}_m$');
%        ylab = ylabel('reference');
%        if strcmp(robotName, 'g500_A')
%        	 tr = title('Robot A: Force-Torque Task Reference');
%        else
%         tr = title('Robot B: Force-Torque Task Reference');
%        end
%        
%     elseif nRow == 3 % only force
%         leg = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$');
%         ylab = ylabel('reference []');
%     end
% 
% end
% 
% if ~strcmp(taskName, 'FORCE_INSERTION') 
%   tr = title(strcat(robotName, " ", taskName));
% end
% set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize );
% set (tr, 'Interpreter', 'latex', 'FontSize', titleFontSize);
% set (ylab, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
% set (xlab, 'Interpreter', 'latex', 'FontSize', xlabFontSize);
% 
% 
% %% plot errors
% % error = importMatrices(strcat(filename, '/error.txt'));
% % nRow = size(refs, 1);
% % nStep = size(refs, 3);
% % 
% % %millisecond indicated in missionManager
% % totSecondPassed = sControlLoop*(nStep-1);
% % seconds = 0:sControlLoop:totSecondPassed;
% % 
% % figure
% % hold on;
% % for i = 1:nRow
% %     a(:) = error(i,1,:);
% %     plot(seconds, a);
% % end
% % xlab = xlabel('time [s]');
% % 
% % if  strcmp(taskName, 'PIPE_REACHING_GOAL') 
% %     leg = legend('$x_{err}$','$y_{err}$', '$z_{err}$', '$pitch_{err}$', '$yaw_{err}$');
% %     ylab = ylabel('error [m],[rad]');
% % elseif  strcmp(taskName, 'JOINT_LIMIT') 
% %     leg = legend('$q1_{err}$','$q2_{err}$', '$q3_{err}$', '$q4_{err}$');
% %     ylab = ylabel('error [rad]');
% % elseif  strcmp(taskName, 'ARM_SHAPE')
% %      if nRow == 1 % scalar type (norm)
% %         leg = legend('$norm_{err}$');
% %         ylab = ylabel('error (norm)');
% %     else
% %         leg = legend('$q1_{err}$','$q2_{err}$', '$q3_{err}$', '$q4_{err}$');
% %         ylab = ylabel('errors [rad]');
% %      end
% %  elseif strcmp(taskName, 'FORCE_INSERTION')
% %      if nRow == 1 %only force norm
% %         leg = legend('$norm_{err}$');
% %         ylab = ylabel('error (norm)');     
% %         
% %     elseif nRow == 2 %norm of force and torque. BEST METHOD
% %     	 leg = legend('$err_f$', '$err_m$');
% %        ylab = ylabel('error (norm)');
% %        
% %      elseif nRow == 3 % only force
% %         leg = legend('$x_{err}$','$y_{err}$', '$z_{err}$');
% %         ylab = ylabel('error [m]');
% %     end
% % end
% % 
% % te = title(strcat(robotName, " ", taskName));
% % set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
% % set (te, 'Interpreter', 'latex', 'FontSize', titleFontSize);
% % set (ylab, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
% % 
% % hold off;
% 
% 
% %% plot activations
% act = importMatrices(strcat(filename, '/activation.txt'));
% nRow = size(refs, 1);
% nStep = size(refs, 3);
% 
% %millisecond indicated in missionManager
% totSecondPassed = sControlLoop*(nStep-1);
% seconds = 0:sControlLoop:totSecondPassed;
% 
% figure('Renderer', 'painters', 'Position', [0 0 650 500])
% hold on;
% for i = 1:nRow
%     a(:) = act(i,1,:);
%     plot(seconds, a);
%         xlim([secInsertion;180]);
% 
% end
% xlab = xlabel('time [s]');
% 
% if  strcmp(taskName, 'JOINT_LIMIT')   
%     leg = legend('$A_{11}$','$A_{22}$', '$A_{33}$', '$A_{44}$');
%     ylab = ylabel('activations');
% elseif strcmp(taskName, 'HORIZONTAL_ATTITUDE')
%     leg = legend('$A$');
%     ylab = ylabel('activation'); 
% elseif strcmp(taskName, 'ARM_SHAPE')
%     if nRow == 1 % scalar type (norm)
%         leg = legend('$A$');
%         ylab = ylabel('activation');
%     else
%       leg = legend('$A_{11}$','$A_{22}$', '$A_{33}$', '$A_{44}$');
%       ylab = ylabel('activations');
%     end
% elseif strcmp(taskName, 'FORCE_INSERTION')
%     if nRow == 1 % only norm force
%         leg = legend('$A$');
%         ylab = ylabel('activation');
%         error (norm)
%         
%   elseif nRow == 2 %norm of force and torque. BEST METHOD
%     	 leg = legend('$a_f$', '$a_m$');
%        ylab = ylabel('activations');
%        if strcmp(robotName, 'g500_A')
%        	 ta = title('Robot A: Force-Torque Task Activation');
%        else
%          ta = title('Robot B: Force-Torque Task Activation');
%        end
%        
% 	elseif nRow == 3 % only force
%         leg = legend('$A_{11}$','$A_{22}$', '$A_{33}$');
%         ylab = ylabel('activations');
%     end
% end
% 
% if ~strcmp(taskName, 'FORCE_INSERTION') 
%   ta = title(strcat(robotName, " " ,taskName));
% end
% set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
% set (ta, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
% set (ylab, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
% set (xlab, 'Interpreter', 'latex', 'FontSize', xlabFontSize);


%% plot activation and ref for force torque divided
if  strcmp(taskName, 'FORCE_INSERTION')   
  actSqueez = squeeze(act);
  
  figure('Renderer', 'painters', 'Position', [0 0 710 550])

  subplot(2,1,1)
  plot(seconds, actSqueez(1,:));
  xlaba = xlabel('time [s]');
  ylaba= ylabel('Activation');
  if strcmp(robotName, 'g500_A')
      ta = title('Robot A: Force-Torque Task Activation - Force part');
   else
     ta = title('Robot B: Force-Torque Task Activation - Force part');
  end
    xlim([secInsertion;180]);

  set (ta, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
  set (ylaba, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
  set (xlaba, 'Interpreter', 'latex', 'FontSize', xlabFontSize);
  
  subplot(2,1,2)
   plot(seconds, actSqueez(2,:));
  xlaba = xlabel('time [s]');
  ylaba= ylabel('Activation');
  if strcmp(robotName, 'g500_A')
      ta = title('Robot A: Force-Torque Task Activation - Torque part');
   else
     ta = title('Robot B: Force-Torque Task Activation - Torque part');
  end
    xlim([secInsertion;180]);

  set (ta, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
  set (ylaba, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
  set (xlaba, 'Interpreter', 'latex', 'FontSize', xlabFontSize);
  
  refSqueez = squeeze(refs);
    figure('Renderer', 'painters', 'Position', [0 0 710 550])

  subplot(2,1,1)
  plot(seconds, refSqueez(1,:));
  xlaba = xlabel('time [s]');
  ylaba= ylabel('Reference');
  if strcmp(robotName, 'g500_A')
      ta = title('Robot A: Force-Torque Task Reference - Force part');
   else
     ta = title('Robot B: Force-Torque Task Reference - Force part');
  end
    xlim([secInsertion;180]);

  set (ta, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
  set (ylaba, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
  set (xlaba, 'Interpreter', 'latex', 'FontSize', xlabFontSize);
  
  subplot(2,1,2)
   plot(seconds, refSqueez(2,:));
  xlaba = xlabel('time [s]');
  ylaba= ylabel('Reference');
  if strcmp(robotName, 'g500_A')
      ta = title('Robot A: Force-Torque Task Reference - Torque part');
   else
     ta = title('Robot B: Force-Torque Task Reference - Torque part');
  end
    xlim([secInsertion;180]);

  set (ta, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
  set (ylaba, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
  set (xlaba, 'Interpreter', 'latex', 'FontSize', xlabFontSize);
  
  
end

%% plot jacob-1*ref (for ydot generated by the single task) only for force insertion for now
legFontSize = 19;
titleFontSize = 18;
ylabFontSize = 17;
xlabFontSize = 17;

if  strcmp(taskName, 'FORCE_INSERTION')   
  jacobs = importMatrices(strcat(rootPath, robotName, '/forceJacob.txt'));
  yDotSingle = zeros(10,nStep);

  for i=1:nStep
    yDotSingle(:,i) = pinv(jacobs(:,:,i)) * refs(:,:,i);  
  end
  
  figure('Renderer', 'painters', 'Position', [0 0 750 990])
  subplot(3,1,1)
  plot (seconds, yDotSingle(1:4,:))
  xlab1 = xlabel('time [s]');
  leg1 = legend('$\dot{q}_1$','$\dot{q}_2$', '$\dot{q}_3$', '$\dot{q}_4$');
  ylab1 = ylabel('Joint velocities [rad/s]');
  if strcmp(robotName, 'g500_A')
   tq1 = title('Joint command for Robot A');
  else
    tq1 = title('Joint command for Robot B');
  end
    xlim([secInsertion;180]);

    set(leg1, 'Interpreter', 'latex', 'FontSize' , legFontSize);
  set (tq1, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
  set (ylab1, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
  set (xlab1, 'Interpreter', 'latex', 'FontSize', xlabFontSize);
  
  subplot(3,1,2);
  plot (seconds, yDotSingle(5:7,:))
  xlab2 = xlabel('time [s]');
  leg2 = legend('$\dot{x}$','$\dot{y}$', '$\dot{z}$');
  ylab2 = ylabel('Linear velocity [m/s]');
  if strcmp(robotName, 'g500_A')
   tq2 = title('Vehicle linear velocity command for Robot A');
  else
    tq2 = title('Vehicle linear velocity command for Robot B');
  end
    xlim([secInsertion;180]);

  
  set(leg2, 'Interpreter', 'latex', 'FontSize' , legFontSize);
  set (tq2, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
  set (ylab2, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
  set (xlab2, 'Interpreter', 'latex', 'FontSize', xlabFontSize);


  subplot(3,1,3);
  plot (seconds, yDotSingle(8:10,:))
  xlab3 = xlabel('time [s]');
  leg3 = legend('$w_x$', '$w_y$', '$w_z$');
  ylab3 = ylabel('Angular velocity [rad/s]');
  if strcmp(robotName, 'g500_A')
   tq3 = title('Vehicle angular velocity command for Robot A');
  else
    tq3 = title('Vehicle angular velocity command for Robot B');
  end
    xlim([secInsertion;180]);


  set(leg3, 'Interpreter', 'latex', 'FontSize' , legFontSize);
  set (tq3, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
  set (ylab3, 'Interpreter', 'latex', 'FontSize', ylabFontSize);
  set (xlab3, 'Interpreter', 'latex', 'FontSize', xlabFontSize);
  
end



