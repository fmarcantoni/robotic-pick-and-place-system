classdef livePlotHandle < handle
    
    properties
        degPlotArray;
        posPlotArray;
        velPlotArray;
        angVelPlotArray;
        magVelPlotArray;
        pathArray;
        degIndex;
        posIndex;
        velIndex;
        magVelIndex;
        angVelIndex;
        pathIndex;
        startTime;
        havePause;
        havePath;
        haveBody;
        haveVelocity;
    end
    
    methods
        function self = livePlotHandle()
           self.degPlotArray = zeros(5,0);
           self.posPlotArray = zeros(4,0);
           self.velPlotArray = zeros(4,0);
           self.angVelPlotArray = zeros(5,0);
           self.magVelPlotArray = zeros(2,0);
           self.pathArray = zeros(3,0);
           self.degIndex = 1;
           self.posIndex = 1;
           self.velIndex = 1;
           self.magVelIndex = 1;
           self.angVelIndex = 1;
           self.pathIndex = 1;
           self.startTime = 0;
           self.havePause = true;
           self.haveBody = true;
           self.havePath = true;
           self.haveVelocity = true;
        end
    end
end

