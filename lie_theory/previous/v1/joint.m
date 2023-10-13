classdef joint < special_euclidean_group
    %special_euclidean_group Summary of this class goes here
    %   Detailed explanation goes here

    properties
        algebra = zeros([4,4]);
        algebra_adj = zeros([4,4]);
        adj = eye(4);
        screw = zeros([6,1]);
    end

    methods
        function obj = joint(screw_joint)
            %joint Construct an instance of this class
            %   Detailed explanation goes here
            obj.screw = screw_joint;
            obj.algebra = vector_to_algebra(obj);
        end

        function obj = update_manifold(obj,motion)
            %algebra_to_manifold Summary of this method goes here
            %   Detailed explanation goes here
            obj.algebra_adj = vector_to_algebra(obj.adj*obj.screw);
            obj.manifold = expm(obj.algebra_adj*motion);
        end

        function obj = algebra_to_manifold(obj,motion)
            %algebra_to_manifold Summary of this method goes here
            %   Detailed explanation goes here
            obj.manifold = expm(obj.algebra*motion);
        end

        function obj = set_adjoint(obj,adjoint)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.adj = adjoint;
        end

%         function obj = manifold_to_algebra(obj,m)
%             %manifold_to_algebra Summary of this method goes here
%             %   Detailed explanation goes here
%             obj.algebra = logm(m);
%         end

%         function obj = algebra_to_vector(obj,a)
%             %algebra_to_vector Summary of this method goes here
%             %   Detailed explanation goes here
%             omega_skew = a(1:3,1:3);
%             rho = a(1:3,4);
%             omega = [omega_skew(3,2);
%                      omega_skew(1,3);
%                      omega_skew(2,1)];
%             obj.vector = [omega,rho];
%         end

        function algebra = vector_to_algebra(obj)
            %vector_to_algebra Summary of this method goes here
            %   Detailed explanation goes here
            omega_skew = [0,-obj.screw(3),obj.screw(2);
                          obj.screw(3),0,-obj.screw(1);
                          -obj.screw(2),obj.screw(1),0];
            rho = [obj.screw(4);
                   obj.screw(5);
                   obj.screw(6)];
            algebra = zeros([4,4]);
            algebra(1:3,1:3) = omega_skew;
            algebra(1:3,4) = rho;
        end

%         function obj = vector_to_manifold(obj,motion)
%             %vector_to_manifold Summary of this method goes here
%             %   Detailed explanation goes here
%             a = vector_to_algebra(obj,motion);
%             obj.manifold = algebra_to_manifold(obj,a);
%         end

%         function obj = manifold_to_vector(obj,m)
%             %manifold_to_vector Summary of this method goes here
%             %   Detailed explanation goes here
%             obj.algebra = manifold_to_algebra(obj,m);
%             obj.vector = algebra_to_vector(obj,obj.algebra);
%         end

%         function outputArg = method1(obj,inputArg)
%             %METHOD1 Summary of this method goes here
%             %   Detailed explanation goes here
%             outputArg = obj.Property1 + inputArg;
%         end
    end
end