classdef NegativeCharge
    properties
        x
        y
        Q = -20; % Magnitud de la carga (coulombs)
    end
    
    methods
        function obj = NegativeCharge(x, y)
            obj.x = x;
            obj.y = y;
        end
        
        function draw(obj)
            plot(obj.x, obj.y, 'bo', 'MarkerSize', 15, 'MarkerFaceColor', 'b');
        end
    end
end
