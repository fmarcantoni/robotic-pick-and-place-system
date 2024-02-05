 tt = 1;
robot = Robot();
robot.writeTime(tt);
colibrate = false;
takePoints = false;
run1 = false;
run2 = false;
run3 = false;
run4 = false;
run5 = true;
run6 = false;

%Calibration of Camera
if colibrate
    robot.servo_jp(robot.fold);
    cc = Camera;
    save('cameraObject.mat','cc');
else
    camStruct = load('cameraObject.mat');
    cc = camStruct.cc;
end
od = objectDetect(cc);

%Taking Points of Image
if takePoints
    imtool(cc.getImage);
end

%Board pixal infoamtion

%Off-set for X & Y axis
xoff = 20;
yoff = 20;

%Edges of the Board to Keep
edgePoints = [
    345 - xoff, 250 - yoff; %top left 
    775 + xoff, 250 - yoff; %top right 
    920 + xoff, 450 + yoff  %bottom right
    200 - xoff, 450 + yoff; %bottom left
    ];
   
imshow(od.findGreen(od.applyMask(cc.getImage,edgePoints)));
Rrange = [20 50];

while run2
    image = imsharpen(cc.getImage);
    om = od.findOrange(image);
    ym = od.findYellow(image);
    gm = od.findGreen(image);
    rm = od.findRed(image);
    [Pball, Rball] = od.getBall(rm  ,Rrange);
    imshow(ym);
    hold on;
%     if(height(Pball) > 0)
% 
%         scatter(Pball(:,1),Pball(:,2),Rball*50,'cyan');
%         scatter(Pball(:,1),Pball(:,2),1000,'cyan','Marker','+');
%     end
%     [Pball, Rball] = od.getBall(gm  ,Rrange);
%     if(height(Pball) > 0)
%         scatter(Pball(:,1),Pball(:,2),Rball*50,'red');
%         scatter(Pball(:,1),Pball(:,2),1000,'red','Marker','+');
%     end
    [Pball, Rball] = od.getBall(ym  ,Rrange);
    if(height(Pball) > 0)
        scatter(Pball(:,1),Pball(:,2),Rball*50,'magenta');
        scatter(Pball(:,1),Pball(:,2),1000,'magenta','Marker','+');
    end
%     [Pball, Rball] = od.getBall(om  ,Rrange);
%     if(height(Pball) > 0)
%         scatter(Pball(:,1),Pball(:,2),Rball*50,'green');
%         scatter(Pball(:,1),Pball(:,2),1000,'green','Marker','+');
%     end
    hold off;
    pause(1/6000);
end

captureRate = 0;
if run3
for i = 1:20
    image = od.applyMask(cc.getImage,edgePoints);
    mask = od.findOrange(image);
    [Pball,Rball] = od.getBall(mask,Rrange);
    imshow(image);
    if(height(Pball) > 0)
        captureRate = captureRate + 1;
    end
    
    hold on;
    if(height(Pball) > 0)
        scatter(Pball(:,1),Pball(:,2),Rball*50,'green');
        scatter(Pball(:,1),Pball(:,2),1000,'green','Marker','+');
    end
    hold off;
    pause(1/4);
end
captureRate
end


if run4
    robot.restPoint;
    pause(tt);
    liftHeight = 25 * 3;
    pickHeight = 25 * 0.6;
    openPercent = 0.35;
    closePercent = 0.1;
    image = imsharpen(od.applyMask(cc.getImage,edgePoints));
    mask = od.findRed(image);
    [Pball,Rball] = od.getBall(mask,Rrange);
    imshow(image);
    hold on;
    if(height(Pball) > 0)
        scatter(Pball(:,1),Pball(:,2),Rball*50,'green');
        scatter(Pball(:,1),Pball(:,2),1000,'green','Marker','+');
    end
    hold off;
    if(height(Pball) > 0)
        realXY = od.ballXYonBoard(Pball);

        liftPoint = od.switchToRobot([realXY -liftHeight],90);
        pickPoint = od.switchToRobot([realXY -pickHeight],90);
        liftJp = robot.ik3001(liftPoint);
        pickJp = robot.ik3001(pickPoint);
        robot.gripperOpenPercent(openPercent);
        pause(tt);
        robot.servo_jp(liftJp)
        pause(tt);
        robot.servo_jp(pickJp);
        pause(tt);
        robot.gripperOpenPercent(closePercent);
        robot.servo_jp(liftJp);
        pause(tt);
        robot.gripperOpenPercent(openPercent);
        pause(tt);
    end
end


%Main Project: Grabing 4 color balls (Yellow, Orange, Red, Green)
if run5

    %Constant Variables
    tt = 0.75;
    robot.writeTime(tt);
    unit = 25;
    liftHeight = unit * 4;
    pickHeight = unit * 0.5;
    openPercent = 0.5;
    closePercent = 0.1;
    robot.restPoint;
    groupWidth = 2;
    pause(tt);
end

while run5
    [Probot, groupIndex] = od.getNextBallPosition(edgePoints,robot.rest,liftHeight,Rrange);
    if(groupIndex > 0)
        %Locaing ball center & move arm to location
        liftJp = robot.ik3001(Probot);
        pickJp = robot.ik3001([Probot(1:2) pickHeight Probot(4)]);
        robot.restPoint;
        pause(tt);

        %Grabing ball
        robot.gripperOpenPercent(openPercent);
        robot.servo_jp(liftJp);
        pause(tt);
        robot.servo_jp(pickJp);
        pause(tt);
        robot.gripperOpenPercent(closePercent);
        pause(tt);
        robot.servo_jp(liftJp);
        pause(tt);
        disp(groupIndex);

        %Placing Ball Down
        dropPoint = [(groupIndex - 1) * groupWidth * unit, -10 * 25, 2 * unit, 45];
        landPoint = [(groupIndex - 1) * groupWidth * unit, -10 * 25, 0.5 * unit, 45];
        setPoint = [(groupIndex - 1) * groupWidth * unit, -11 * 25, 0.5 * unit, 45];
        robot.servo_jp(robot.ik3001(dropPoint));
        pause(tt);
        robot.servo_jp(robot.ik3001(landPoint));
        pause(tt);  
        robot.servo_jp(robot.ik3001(setPoint));
        pause(tt);
        robot.gripperOpenPercent(openPercent);
        pause(tt);
        robot.servo_jp(robot.ik3001(dropPoint));
        pause(tt);
        robot.restPoint;
        pause(tt);
    else
        pause(1/6000);
    end
end

%Live Tracking: Extra Credit (tracking Green Ball)
if run6
    tt = 0.01;
    robot.writeTime(tt,0);
    tic;
end
while run6
    %Take Image
    image = cc.getImage;
    image = imsharpen(image);
    image = od.applyMask(image,edgePoints);
    gm = od.findGreen(image);
    imshow(gm);

    %Find Ball (This case Green)
    [Pball, Rball] = od.getBall(gm,Rrange);
    if height(Pball) > 0 && height(Pball) < 2

        %Ploting information
        hold on;
        scatter(Pball(:,1),Pball(:,2),Rball*50,'green');
        scatter(Pball(:,1),Pball(:,2),1000,'green','Marker','+');
        hold off;
        %Move arm to ball (Green ball) center
        realXY = od.ballXYonBoard(Pball);
        trackPoint = od.switchToRobot([realXY -3 * 25],90);
        if toc > tt
            robot.servo_jp(robot.ik3001(trackPoint));
            tic;
        end
    end                                        
end




% p1 = [3 * 25 0 3 * 25 90];
% p2 = [6 * 25 4 * 25 3 * 25 90];
% p3 = [6 * 25 -4 * 25 3* 25 90];
% 
%  coef1 = traj.quintic_traj(0,timeInterval,0,0,p1(1),p2(1),0,0); 
%  coef2 = traj.quintic_traj(0,timeInterval,0,0,p1(2),p2(2),0,0);
%  coef3 = traj.quintic_traj(0,timeInterval,0,0,p1(3),p2(3),0,0);
%  coef4 = traj.quintic_traj(0,timeInterval,0,0,p1(4),p2(4),0,0);
%  coefs = [coef1 coef2 coef3 coe+f4];
%  robot.run_trajectory(coefs,timeInterval,jointSpace,cubic);

