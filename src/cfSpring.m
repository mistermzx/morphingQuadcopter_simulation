classdef cfSpring
    properties
        s_SH; %in body frame
        s_MH; %in arm frame
        %s_MS; %in A_frame
        fs;
        T_AS;
    end
    
    methods
        function obj = cfSpring(SH, MH, force)
            obj.s_SH = SH;
            obj.s_MH = MH;
            obj.fs = force;
        end
    end
end