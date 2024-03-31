function [] = PlotRobotJoint(TrSimTime,SimTheta,SimDTheta,SimDDTheta)

subplot(321)
plot(TrSimTime,SimTheta(:,1),'r')
hold on, grid minor
plot(TrSimTime,SimDTheta(:,1),'g')
plot(TrSimTime,SimDDTheta(:,1),'b')
ylabel('Joint 1', 'Fontsize', 12);
xlabel('t (second)')
title('Trapezoidal Velocity Profile')
legend('$\theta(t) (Degree)$','$\dot{\theta}(t) (Degree/Sn)$','$\ddot{\theta}(t) (Degree/Sn^2)$','Interpreter','latex','fontsize',8)
legend('boxoff')
axis auto

subplot(322)
plot(TrSimTime,SimTheta(:,2),'r')
hold on, grid minor
plot(TrSimTime,SimDTheta(:,2),'g')
plot(TrSimTime,SimDDTheta(:,2),'b')
ylabel('Joint 2', 'Fontsize', 12);
xlabel('t (second)')
title('Trapezoidal Velocity Profile')
legend('$\theta(t) (Degree)$','$\dot{\theta}(t) (Degree/Sn)$','$\ddot{\theta}(t) (Degree/Sn^2)$','Interpreter','latex','fontsize',8)
legend('boxoff')
axis auto

subplot(323)
plot(TrSimTime,SimTheta(:,3),'r')
hold on, grid minor
plot(TrSimTime,SimDTheta(:,3),'g')
plot(TrSimTime,SimDDTheta(:,3),'b')
ylabel('Joint 3', 'Fontsize', 12);
xlabel('t (second)')
title('Trapezoidal Velocity Profile')
legend('$\theta(t) (Degree)$','$\dot{\theta}(t) (Degree/Sn)$','$\ddot{\theta}(t) (Degree/Sn^2)$','Interpreter','latex','fontsize',8)
legend('boxoff')
axis auto

subplot(324)
plot(TrSimTime,SimTheta(:,4),'r')
hold on, grid minor
plot(TrSimTime,SimDTheta(:,4),'g')
plot(TrSimTime,SimDDTheta(:,4),'b')
ylabel('Joint 4', 'Fontsize', 12);
xlabel('t (second)')
title('Trapezoidal Velocity Profile')
legend('$\theta(t) (Degree)$','$\dot{\theta}(t) (Degree/Sn)$','$\ddot{\theta}(t) (Degree/Sn^2)$','Interpreter','latex','fontsize',8)
legend('boxoff')
axis auto

subplot(325)
plot(TrSimTime,SimTheta(:,5),'r')
hold on, grid minor
plot(TrSimTime,SimDTheta(:,5),'g')
plot(TrSimTime,SimDDTheta(:,5),'b')
ylabel('Joint 5', 'Fontsize', 12);
xlabel('t (second)')
title('Trapezoidal Velocity Profile')
legend('$\theta(t) (Degree)$','$\dot{\theta}(t) (Degree/Sn)$','$\ddot{\theta}(t) (Degree/Sn^2)$','Interpreter','latex','fontsize',8)
legend('boxoff')
axis auto

subplot(326)
plot(TrSimTime,SimTheta(:,6),'r')
hold on, grid minor
plot(TrSimTime,SimDTheta(:,6),'g')
plot(TrSimTime,SimDDTheta(:,6),'b')
ylabel('Joint 6', 'Fontsize', 12);
xlabel('t (second)')
title('Trapezoidal Velocity Profile')
legend('$\theta(t) (Degree)$','$\dot{\theta}(t) (Degree/Sn)$','$\ddot{\theta}(t) (Degree/Sn^2)$','Interpreter','latex','fontsize',8)
legend('boxoff')
axis auto
end