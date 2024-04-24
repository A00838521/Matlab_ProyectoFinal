classdef particulas
    properties
        x % x-coordinate
        y % y-coordinate
        charge % charge of the particle
        cantidad % number of particles
        a
    end
    
    methods
        function obj = particulas(x, y, charge, cantidad, a)
            obj.x = x;
            obj.y = y;
            obj.charge = charge;
            obj.cantidad = cantidad;
            obj.a = a;
        end

        function [Ex, Ey] = getElectricField(obj)
            % Calculate the electric field components Ex and Ey
            % based on the particle's charge and position
            
            % Constantes
            eps0 = 8.85e-12; % Permitividad del espacio libre
            kC = 1/(4*pi*eps0); % Constante de Coulomb
            
            % Initialize electric field components
            Ex = 0;
            Ey = 0;
            
            % Calculate electric field components for each particle
            for i = 1:obj.cantidad
                % Calculate distance between particle and origin
                r = sqrt(obj.x(i)^2 + obj.y(i)^2);
                
                % Calculate electric field components
                Ex = Ex + (kC * obj.charge(i) * obj.x(i)) / r^3;
                Ey = Ey + (kC * obj.charge(i) * obj.y(i)) / r^3;
            end
        end
        function plotParticle(obj)
            for i = 1:obj.cantidad
                h=rectangle('Position',[obj.x(i)-obj.a/2,obj.y(i)-obj.a/2,obj.a,obj.a],'curvature',[1 1]); 
                if obj.charge > 0
                    set(h,'Facecolor',[1 0 0],'Edgecolor',[1 0 1]);
                    text(obj.x(i)-0.1,obj.y(i),'+','Color','white','FontSize',30);
                else
                    set(h,'Facecolor',[0 0 1],'Edgecolor',[0 0 1]);
                    text(obj.x(i)-0.07,obj.y(i)+0.05,'-','Color','white','FontSize',30);
                end
                hold on
            end
        end

    end
end