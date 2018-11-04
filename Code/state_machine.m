classdef state_machine
    %STATE_MACHINE Summary of this class goes here
    %   Detailed explanation goes here

    enumeration
        Idle, Takeoff, Hover, Tracking, Land
    end

    properties
        Property1
    end
    
    methods
        function obj = state_machine(inputArg1,inputArg2)
            %STATE_MACHINE Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

