classdef screw_joint
    %screw_joint Summary of this class goes here
    %   Detailed explanation goes here

    properties
        omega = zeros([3,1])
        v = zeros([3,1])
        d = zeros([3,1])

        S = zeros([6,1])
    end

    methods
        function obj = screw_joint(rotational_velocity,linear_velocity,translation)
            %screw_joint Construct an instance of this class
            %   Detailed explanation goes here
            obj.omega = rotational_velocity/norm(rotational_velocity);
            obj.v = linear_velocity/norm(linear_velocity);
            obj.d = translation;

            obj.S(1:3,1) = obj.omega;
            obj.S(4:6,1) = obj.v + cross(obj.d,obj.omega);
        end

%         function outputArg = method1(obj,inputArg)
%             %METHOD1 Summary of this method goes here
%             %   Detailed explanation goes here
%             outputArg = obj.Property1 + inputArg;
%         end
    end
end