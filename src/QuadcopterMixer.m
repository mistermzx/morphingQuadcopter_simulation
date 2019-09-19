classdef QuadcopterMixer
    properties
        mass;
        inertiaMatrix;
        mixerMatrix;
    end
    
    methods 
        function obj = QuadcopterMixer(mass_quad, inertia, armLength, thrustToTorque)
            obj.mass = mass_quad;
            obj.inertiaMatrix = inertia;
            
            l = armLength;
            k = thrustToTorque;
            M = [ 1 1 1 1;...
                0 l 0 -l;...
                -l 0 l 0;...
                -k +k -k +k];
            obj.mixerMatrix = inv(M);
        end
        
        function motForceCmd = get_motor_force_cmd(obj, desNormThrust, desAngAcc)
            % Calculate motorForce cmd
            ftot = obj.mass*norm(desNormThrust);
            moments = obj.inertiaMatrix*desAngAcc;
            
            motForceCmd = obj.mixerMatrix*[ftot, moments(1), moments(2), moments(3)]';
            
            for i = 1:4
                if(motForceCmd(i)<0)
                    motForceCmd(i)=0;
                end
            end
        end
    end
end