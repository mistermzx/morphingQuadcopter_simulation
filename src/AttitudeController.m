classdef AttitudeController
    properties
        timeConstAngle_RP;
        timeConstAngle_Y;
        timeConstRate_RP;
        timeConstRate_Y;
    end
    
    methods
        function obj = AttitudeController(timeConstant_angle_rollPitch, timeConstant_angle_yaw,timeConstant_rate_rollPitch,timeConstant_rate_yaw)
            obj.timeConstAngle_RP = timeConstant_angle_rollPitch;
            obj.timeConstAngle_Y = timeConstant_angle_yaw;
            obj.timeConstRate_RP = timeConstant_rate_rollPitch;
            obj.timeConstRate_Y = timeConstant_rate_yaw;
        end
        
        function angAccDes = get_angular_acceleration(obj, desNormThrust, curAtt, curAngVel)
            % Computes the desired angular velocity
            
            % Construct a desired attitude, that matches the desired thrust
            desThrustDir = desNormThrust/norm(desNormThrust);
            
            e3 = [0, 0, 1]';
            angle = acos(dot(desThrustDir,e3));
            rotAx = cross(e3, desThrustDir);
            n = norm(rotAx);
            if n<1e-6
                desAtt = Rotation();
            else
                desAtt = Rotation.from_rotation_vector(rotAx*(angle/n));
            end
            
            % Compute desired rates:
            inverse = curAtt.inverse();
            desRotVec = desAtt.compose(inverse).to_rotation_vector();
            
            desAngVel = zeros(3,1);
            desAngVel(1) = desRotVec(1)/obj.timeConstAngle_RP;
            desAngVel(2) = desRotVec(2)/obj.timeConstAngle_RP;
            desAngVel(3) = desRotVec(3)/obj.timeConstAngle_Y;
            
            % Compute desired angular acceleration
            angAccDes = desAngVel-curAngVel;
            angAccDes(1) = angAccDes(1)/obj.timeConstRate_RP;
            angAccDes(2) = angAccDes(2)/obj.timeConstRate_RP;
            angAccDes(3) = angAccDes(3)/obj.timeConstRate_Y;
        end
    end
end