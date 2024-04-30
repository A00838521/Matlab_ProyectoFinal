classdef PositiveCharge
    properties
        x
        y
        Q = 20; % Magnitud de la carga (coulombs)
    end
    
    methods
        function obj = PositiveCharge(x, y)
            obj.x = x;
            obj.y = y;
        end
        
        function draw(obj)
            plot(obj.x, obj.y, 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
        end
    end
end
