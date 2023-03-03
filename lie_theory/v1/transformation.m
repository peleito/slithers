classdef transformation < special_euclidean_group
    %special_euclidean_group Summary of this class goes here
    %   Detailed explanation goes here

    properties
        algebra = zeros([4,4]);
        vector = zeros([6,1]);
        manifold = eye(4);
    end

    methods
        function obj = transformation(inputArg1,inputArg2)
            %transformation Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end

        function obj = algebra_to_manifold(obj,a)
            %algebra_to_manifold Summary of this method goes here
            %   Detailed explanation goes here
            obj.manifold = expm(a);
        end

        function obj = manifold_to_algebra(obj,m)
            %manifold_to_algebra Summary of this method goes here
            %   Detailed explanation goes here
            obj.algebra = logm(m);
        end

        function obj = algebra_to_vector(obj,a)
            %algebra_to_vector Summary of this method goes here
            %   Detailed explanation goes here
            omega_skew = a(1:3,1:3);
            rho = a(1:3,4);
            omega = [omega_skew(3,2);
                     omega_skew(1,3);
                     omega_skew(2,1)];
            obj.vector = [omega,rho];
        end

        function obj = vector_to_algebra(obj,v)
            %vector_to_algebra Summary of this method goes here
            %   Detailed explanation goes here
            omega_skew = [0,-v(3),v(2);
                          v(3),0,-v(1);
                          -v(2),v(1),0];
            rho = [v(4);
                   v(5);
                   v(6)];
            obj.algebra = zeros([4,4]);
            obj.algebra(1:3,1:3) = omega_skew;
            obj.algebra(1:3,4) = rho;
        end

        function obj = vector_to_manifold(obj,v)
            %vector_to_manifold Summary of this method goes here
            %   Detailed explanation goes here
            obj.algebra = vector_to_algebra(obj,v);
            obj.manifold = algebra_to_manifold(obj,obj.algebra);
        end

        function obj = manifold_to_vector(obj,m)
            %manifold_to_vector Summary of this method goes here
            %   Detailed explanation goes here
            obj.algebra = manifold_to_algebra(obj,m);
            obj.vector = algebra_to_vector(obj,obj.algebra);
        end

%         function outputArg = method1(obj,inputArg)
%             %METHOD1 Summary of this method goes here
%             %   Detailed explanation goes here
%             outputArg = obj.Property1 + inputArg;
%         end
    end
end