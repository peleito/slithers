classdef kinematic_chain
    %kinematic_chain Summary of this class goes here
    %   Detailed explanation goes here

    properties
        joints;
        config_state;
        n;
        base_pose;
    end

    methods
        function obj = kinematic_chain(screws, initial)
            %kinematic_chain Construct an instance of this class
            %   Detailed explanation goes here
            num = 0;
            for screw = screws
                num = num+1;
                obj.joints(num) = joint(screw);
            end
            obj.n = num;
            obj.config_state = initial;
        end

        function pose = forward_kinematics(obj,num)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            pose = eye(4);

            for i = 1:1:num
                pose = pose*obj.joints(i).manifold;
            end
            pose = pose*obj.config_state;
        end

        function [tau_poses, tau_joints] = inverse_kinematics(obj,pose)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            poses = special_euclidean_group(pose*inv(obj.config_state));
            tau_poses = poses.manifold_to_vector(poses);
            joints_solved = eye(4);
            for i = 1:1:obj.n
                obj.joints(i).set_adjoint(obj.joints(i),obj.base_pose)
                obj.joints(i).update_manifold(obj.joints(i),dq)
                joints_solved = joints_solved*obj.joints(i).manifold;
            end
            tau_joints = joints_s.manifold_to_vector;
            outputArg = obj.Property1 + inputArg;
        end
        
        function update_motion(obj,dq)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
%             pose = eye(4);

            for i = 1:1:num
                obj.joints(i).set_adjoint(obj.joints(i),obj.base_pose)
                obj.joints(i).update_manifold(obj.joints(i),dq(i))
%                 pose = pose*obj.joints(i).manifold;
            end
%             pose = pose*obj.config_state;
        end

        function obj = set_base_pose(obj,pose)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.base_pose = pose;
        end

%         function outputArg = method1(obj,inputArg)
%             %METHOD1 Summary of this method goes here
%             %   Detailed explanation goes here
%             outputArg = obj.Property1 + inputArg;
%         end
    end
end