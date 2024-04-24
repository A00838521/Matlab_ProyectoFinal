% Crear grid
clc; clear; close all; % Limpiar la consola, las variables y las figuras
N=20; % Número de puntos en cada dimensión
minX=-2; maxX=+2; % Valores mínimos y máximos de x
minY=-2; maxY=+2; % Valores mínimos y máximos de y
x=linspace(minX,maxX,N); % Generar valores de x
y=linspace(minY,maxY,N); % Generar valores de y
[xG,yG]=meshgrid(x,y); % Crear una cuadrícula de puntos en el espacio 2D

NQn = 3;
NQp = 3;

% Información de carga negativa
% xCn = ones(1, NQn);
% yCn = linspace(0, (NQn-1)*a, NQn);
xCn = 0; % Coordenada x de la partícula negativa
yCn = 0; % Coordenada y de la partícula negativa
Qn = -20; % Carga negativa de la partícula

% Posición de la partícula positiva
xCp =1; % Coordenada x de la partícula positiva
yCp = 0; % Coordenada y de la partícula positiva
Qp = 20; % Carga positiva de la partícula

% Posición de la partícula central
xC = 0; % Coordenada x de la partícula central
yC = 0; % Coordenada y de la partícula central

% Ancho de las partículas
a=0.4; % Ancho de la partícula
aParticula=0.1; % Ancho de la partícula central

% Constantes
eps0 = 8.85e-12; % Permitividad del espacio libre
kC = 1/(4*pi*eps0); % Constante de Coulomb


% Graficar la flecha
% figure();

% % Graficar la partícula central
% h=rectangle('Position',[xC-aParticula/2,yC-aParticula/2,aParticula,aParticula],'curvature',[1 1]); 
% set(h,'Facecolor',[0 0 0],'Edgecolor',[0 0 1]);

u = zeros(size(xG));
v = zeros(size(yG));

% Graficar la partícula negativa
for i=1:NQn
    for j=1:NQn
        Rx = xG - xCn; % Componente x de la distancia entre cada punto y la partícula negativa
        Ry = yG - yCn - (a * j-1); % Componente y de la distancia entre cada punto y la partícula negativa
        R = sqrt(Rx.^2 + Ry.^2).^3; % Distancia entre cada punto y la partícula negativa
        Ex = kC .* Qn .* Rx ./ R; % Componente x del campo eléctrico debido a la partícula negativa
        Ey = kC .* Qn .* Ry ./ R; % Componente y del campo eléctrico debido a la partícula negativa
        
        u = u + Ex;
        v = v + Ey;
    end
end

% Graficar la partícula positiva
for i=1:NQp
    for j=1:NQp
        Rx = xG - xCp; % Componente x de la distancia entre cada punto y la partícula positiva
        Ry = yG - yCp - (a * j-1); % Componente y de la distancia entre cada punto y la partícula positiva
        R = sqrt(Rx.^2 + Ry.^2).^3; % Distancia entre cada punto y la partícula positiva
        Ex = Ex + kC .* Qp .* Rx ./ R; % Componente x del campo eléctrico debido a la partícula positiva
        Ey = Ey + kC .* Qp .* Ry ./ R; % Componente y del campo eléctrico debido a la partícula positiva
        u = u + Ex;
        v = v + Ey;
    end
end

E = sqrt(u.^2 + v.^2); % Magnitud del campo eléctrico en cada punto

% Componentes u y v
u = u./E; % Componente x del vector unitario del campo eléctrico
v = v./E; % Componente y del vector unitario del campo eléctrico

% Graficar las flechas
% u_avg = mean(u(:)); % Componente x promedio del vector unitario del campo eléctrico
% v_avg = mean(v(:)); % Componente y promedio del vector unitario del campo eléctrico

figure();
quiver(xG, yG, u, v, 'autoscalefactor', 0.6, 'color', [1 0 0], 'linewidth', 1.2);
hold on;

for i=1:NQn
    for j=1:NQn
        plot(xCn * ones(1, NQn) , yCn + a * (0:NQn-1), 'bo', 'MarkerSize', 15, 'MarkerFaceColor', 'b'); % Cargas negativas
        plot(xCp * ones(1, NQp), yCp + a * (0:NQp-1), 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r'); % Cargas positivas
    end
end
axis([-1.5 1.5 -1.5 1.5]);
axis equal;
box on;

% % Cálculo del Campo Eléctrico => Partícula Central 2
% magnitudDeCargas = sqrt(Qn^2 + Qp^2); % Magnitud de las cargas
% d = sqrt((xCn - xCp)^2 + (yCn - yCp)^2); % Distancia entre las partículas negativa y positiva
% p = magnitudDeCargas*d; % Producto de la magnitud de las cargas y la distancia
% % Punto medio
% xm = (xCn + xCp)/2; % Coordenada x del punto medio
% ym = (yCn + yCp)/2; % Coordenada y del punto medio

% % Distancia desde el punto medio hasta la partícula central
% r = sqrt((xC - xm)^2 + (yC - ym)^2); % Distancia entre el punto medio y la partícula central
% vectorPosicionR = (xC - xm) + (yC - ym); % Vector de posición desde el punto medio hasta la partícula central
% magnitudDeR = sqrt((xC - xm)^2 + (yC - ym)^2); % Magnitud del vector de posición
% vectorUnitarioR = vectorPosicionR/magnitudDeR; % Vector unitario en la dirección del vector de posición
% % Campo eléctrico de la partícula central
% E = (kC * (2*p/(r^2)) * vectorUnitarioR); % Campo eléctrico en la partícula central

% % Graficar las flechas
% % u_avg = mean(u(:)); % Componente x promedio del vector unitario del campo eléctrico
% % v_avg = mean(v(:)); % Componente y promedio del vector unitario del campo eléctrico

% h=quiver(xC, yC, u_avg*3, v_avg*3, 'Color', 'blue', 'LineWidth', 2, 'MaxHeadSize', 1.5); % Graficar la flecha en la partícula central
% set(h,'color',[0 0 1],'linewidth',1.2);
% axis equal;
% box on
% hold off;

% % Campo eléctrico de la partícula central
% disp("Coordenadas de la partícula central:"); % Mostrar las coordenadas de la partícula central
% disp("x: " + xC);
% disp("y: " + yC);
% disp("Campo eléctrico de la partícula central:"); % Mostrar el campo eléctrico en la partícula central
% disp("E: " + E + "N/C");