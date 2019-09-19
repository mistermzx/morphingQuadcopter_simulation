classdef PositionController
    properties
        natFreq;
        dampingRatio;
    end
    methods
        function obj = PositionController(naturalFrequency, dampingRatio)
            obj.natFreq = naturalFrequency;
            obj.dampingRatio = dampingRatio;
        end
        
        function accDes = get_acceleration_command (obj, desPos, curPos, curVel)
            % Calcuates accDes based on desPos
            accDes = -2*obj.dampingRatio*obj.natFreq*curVel - obj.natFreq^2*(curPos-desPos);
        end
    end
end