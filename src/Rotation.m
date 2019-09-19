classdef Rotation
    properties 
        q;
    end
    methods (Static)
        function Q = skewSymmetric(q)
            %q is vector
            q0 = q(1);
            qv = q(2:end);
            Qv = [0 -qv(3) qv(2); qv(3) 0 -qv(1); -qv(2) qv(1) 0];
            Q = q0*eye(4) + [0 qv'; -qv Qv];
        end
        function rotation = fromAxisAngle(unitVector, angle)
            rotation = Rotation ();
            rotation.q = [cos(angle/2), sin(angle/2)*unitVector(1), sin(angle/2)*unitVector(2), sin(angle/2)*unitVector(3)]';
        end
        
        function rotation = from_rotation_vector (rotVec)
            theta = norm(rotVec);
            if(theta<4.84813681e-9)
                rotation = Rotation();
                return
            end
            rotation = Rotation();
            rotation = Rotation.fromAxisAngle(rotVec/theta, theta);
        end
        
        function rotation = from_euler_RPY(rpy)
            r = rpy(1);
            p = rpy(2);
            y = rpy(3);
            rotation = Rotation();
            rotation.q = [cos(0.5*y)*cos(0.5*p)*cos(0.5*r) + sin(0.5*y)*sin(0.5*p)*sin(0.5*r),...
                        cos(0.5*y)*cos(0.5*p)*sin(0.5*r) - sin(0.5*y)*sin(0.5*p)*cos(0.5*r),...
                        cos(0.5*y)*sin(0.5*p)*cos(0.5*r) + sin(0.5*y)*cos(0.5*p)*sin(0.5*r),...
                         sin(0.5*y)*cos(0.5*p)*cos(0.5*r) - cos(0.5*y)*sin(0.5*p)*sin(0.5*r)]';
        end
        
    end
    methods
        function obj = Rotation ()
            obj.q = [1, 0, 0, 0]';
        end
       
        function obj = Rotation2 (q0, q1, q2, q3)
             obj.q = [q0, q1, q2, q3]';
        end
        
        function obj = normalize(obj)
            obj.q = obj.q/norm(obj.q);
        end
        
        function T = to_trans_matrix(obj)
            q = obj.q;
            r0=q(1)*q(1);
            r1=q(2)*q(2);
            r2=q(3)*q(3);
            r3=q(4)*q(4);
            T = zeros(3,3);
            T(1,1) = r0 + r1 - r2 - r3;
            T(1,2) = 2*q(2)*q(3) + 2*q(1)*q(4);
            T(1,3) = 2*q(2)*q(4) - 2*q(1)*q(3);
            T(2,1) = 2*q(2)*q(3) - 2*q(1)*q(4);
            T(2,2) = r0-r1+r2-r3;
            T(2,3) = 2*q(3)*q(4) + 2*q(1)*q(2);
            T(3,1) = 2*q(2)*q(4) + 2*q(1)*q(3);
            T(3,2) = 2*q(3)*q(4) - 2*q(1)*q(2);
            T(3,3) = r0-r1-r2+r3;
        end 
    
        function euler = to_euler(obj)
            q = obj.q;
            y = atan2(2*q(2)*q(3) + 2*q(1)*q(4), q(2)*q(2) + q(1)*q(1)- q(4)*q(4) - q(2)*q(2));
            p = -asin(2.0*q(2)*q(4) - 2.0*q(1)*q(3));
            r = atan2(2.0*q(3)*q(4) + 2.0*q(1)*q(2), q(4)*q(4) - q(3)*q(3)- q(2)*q(2) + q(1)*q(1));
            euler = [r, p, y]';
        end
 
        function obj = compose(obj, q_CB)
            % q_CB is rotation object
            %q_CA = Q_CB(input)*q_BA (obj)
            obj.q = transpose(Rotation.skewSymmetric(q_CB.q))*obj.q;
            obj = obj.normalize();
        end
        
        function rotVec = to_rotation_vector(obj)
            n = obj.q(2:end);
            theta = 2*asin(norm(n));
            if (abs(theta)<1e-9)
                rotVec = zeros(3,1);
                return;
            else
                rotVec = theta*n/norm(n);
                return;
            end
        end
        
        function obj = inverse(obj)
            obj.q(2) = - obj.q(2);
            obj.q(3) = - obj.q(3);
            obj.q(4) = - obj.q(4);
        end
    end
end