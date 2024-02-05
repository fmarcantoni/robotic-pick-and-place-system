% travelTime = 5;
% robot = Robot();
% robot.writeMotorState(true);
% robot.interpolate_jp(0,travelTime*1000);
% pause(travelTime);
% positions = zeros(313,5);
% sampleIndex = 1;
% robot.interpolate_jp([45,0,0,0],travelTime*1000);
% tic
% while toc < travelTime
%     positions(sampleIndex,2:5) = robot.setpoint_js;
%     positions(sampleIndex,1) = toc*1000;
%     sampleIndex = sampleIndex + 1;
% end
% 
% subplot(4, 1, 1);
% plot(positions(1:313,1),positions(1:313,2))
% axis([0, 5000, 0, 45])
% title("First Joint")
% xlabel("Time (ms)")
% ylabel("Angle (deg)")
% 
% subplot(4, 1, 2);
% plot(positions(1:313,1),positions(1:313,3))
% axis([0, 5000, 0, 45])
% title("Second Joint")
% xlabel("Time (ms)")
% ylabel("Angle (deg)")
% 
% subplot(4, 1, 3);
% plot(positions(1:313,1),positions(1:313,4))
% axis([0, 5000, 0, 45])
% title("Third Joint")
% xlabel("Time (ms)")
% ylabel("Angle (deg)")
% 
% subplot(4, 1, 4);
% plot(positions(1:313,1),positions(1:313,5))
% axis([0, 5000, 0, 45])
% title("Fourth Joint")
% xlabel("Time (ms)")
% ylabel("Angle (deg)")

%exportgraphics(gcf,"./export/4JointsAngle(interpolaion).png");
% clf
% 
previousTime=0;
timeIntervals = zeros(313,1);
j = 1;
for i = transpose(positions(1:313,1))
     timeIntervals(j) = i - previousTime;
     j = j + 1;
     previousTime = i;
 end
 histogram(timeIntervals);
 title("timing histogram");
 ylabel("Time (ms)");
% %exportgraphics(gcf,"./export/timingHistogram(interpolation).png");

