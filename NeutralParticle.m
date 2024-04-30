classdef NeutralParticle
    properties
        x
        y
    end

    methods
        function obj = NeutralParticle(x, y)
            obj.x = x;
            obj.y = y;
        end

        function draw(obj)
            plot(obj.x, obj.y, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
        end
    end
end
