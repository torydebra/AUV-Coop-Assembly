function plotGenericErrorDivided(pathName)

legFontSize = 13;
titleFontSize = 13;
ylabFontSize = 12;

errors = importMatrices(pathName);
nRow = size(errors, 1);
nStep = size(errors, 3);

global sControlLoop
totSecondPassed = sControlLoop*(nStep-1);
seconds = 0:sControlLoop:totSecondPassed;

%squeeze shrink the 3d matrix into 2d matrix. errors has the second
%dimension of always size 1
errorsLinSqueezed = squeeze(errors(1:3,:,:));
errorsAngSqueezed = squeeze(errors(4:6,:,:));

linNorms = vecnorm(errorsLinSqueezed); %return the norm of each column
angNorms = vecnorm(errorsAngSqueezed);

figure
subplot(2,1,1);
plot(seconds, linNorms);
xlabel('time [s]');
ylab1 = ylabel('errors norm [m]');
tq1 = title("Norm of Cartesian Linear Error");
xlim([0 10]);
ylim([0 0.13]);

subplot(2,1,2);
plot(seconds, angNorms);
xlabel('time [s]');
ylab2 = ylabel('errors norm [rad]');
tq2 = title("Norm of Cartesian Angular Error");
xlim([0 10]);
ylim([0 0.4]);


set(ylab1, 'Interpreter', 'none', 'FontSize' , ylabFontSize);
set (tq1, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
set(ylab2, 'Interpreter', 'none', 'FontSize' , ylabFontSize);
set (tq2, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
hold off






%% plot vel norm
% figure
% hold on;
% for i = 1:nRow
%     a(:) = norms(i,1,:);
%     plot(seconds, a);
% end
% xlabel('time [s]');
% ylab = ylabel('norms');
% 
% tq = title("Norms (lin, ang) of Cartesian Error between the two coincident pipes");
% 
% leg = legend('Linear', 'Angular');
% set(leg, 'Interpreter', 'latex', 'FontSize' , legFontSize);
% set(ylab, 'Interpreter', 'none', 'FontSize' , ylabFontSize);
% 
% set (tq, 'Interpreter', 'latex', 'FontSize' , titleFontSize);
% hold off
