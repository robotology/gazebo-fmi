figure;
subplot(5,1,1);
plot(simTime, inputActuator);
title('Input actuator');
subplot(5,1,2);
plot(simTime, outputActuator);
title('Output actuator');
subplot(5,1,3);
plot(simTime, jointPosition);
title('Joint position');
subplot(5,1,4);
plot(simTime, jointVelocity);
title('Joint velocity');
subplot(5,1,5);
plot(simTime, jointAcceleration);
title('Joint acceleration');

