% Crear grid
clc; clear; close all; % Limpiar la consola, las variables y las figuras
N=20; % Número de puntos en cada dimensión
minX=-2; maxX=+2; % Valores mínimos y máximos de x
minY=-2; maxY=+2; % Valores mínimos y máximos de y
x=linspace(minX,maxX,N); % Generar valores de x
y=linspace(minY,maxY,N); % Generar valores de y
[xG,yG]=meshgrid(x,y); % Crear una cuadrícula de puntos en el espacio 2D

% Información de carga negativa
xCn = 1; % Coordenada x de la partícula negativa
yCn = -1.5; % Coordenada y de la partícula negativa
Qn = 0; % Carga negativa de la partícula
Qn = abs(Qn) * -1; % Asegurar carga negativa5
NQn = 10;

% Posición de la partícula positiva
xCp = 0; % Coordenada x de la partícula positiva
yCp = -1.5; % Coordenada y de la partícula positiva
Qp = 0; % Carga positiva de la partícula
Qp = abs(Qp) * 1; % Asegurar carga positiva
NQp = 10;

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
figure();

% Graficar la partícula central
h=rectangle('Position',[xC-aParticula/2,yC-aParticula/2,aParticula,aParticula],'curvature',[1 1]); 
set(h,'Facecolor',[0 0 0],'Edgecolor',[0 0 1]);

% Graficar la partícula negativa
for i=1:NQn
    h=rectangle('Position',[xCn-a/2,yCn-a/2,a,a],'curvature',[1 1]); 
    set(h,'Facecolor',[0 0 1],'Edgecolor',[0 0 1]);
    text(xCn-0.07,yCn+0.05,'-','Color','white','FontSize',30);
    hold on
    
    Rx = xG - xCn; % Componente x de la distancia entre cada punto y la partícula negativa
    Ry = yG - yCn; % Componente y de la distancia entre cada punto y la partícula negativa
    R = sqrt(Rx.^2 + Ry.^2).^3; % Distancia entre cada punto y la partícula negativa
    Ex = kC .* Qn .* Rx ./ R; % Componente x del campo eléctrico debido a la partícula negativa
    Ey = kC .* Qn .* Ry ./ R; % Componente y del campo eléctrico debido a la partícula negativa
    
    yCn = yCn+a;
end

% Graficar la partícula positiva
for i=1:NQp
    h=rectangle('Position',[xCp-a/2,yCp-a/2,a,a],'curvature',[1 1]); 
    set(h,'Facecolor',[1 0 0],'Edgecolor',[1 0 1]);
    text(xCp-0.1,yCp,'+','Color','white','FontSize',30);
    hold on

    Rx = xG - xCp; % Componente x de la distancia entre cada punto y la partícula positiva
    Ry = yG - yCp; % Componente y de la distancia entre cada punto y la partícula positiva
    R = sqrt(Rx.^2 + Ry.^2).^3; % Distancia entre cada punto y la partícula positiva
    Ex = Ex + kC .* Qp .* Rx ./ R; % Componente x del campo eléctrico debido a la partícula positiva
    Ey = Ey + kC .* Qp .* Ry ./ R; % Componente y del campo eléctrico debido a la partícula positiva

    yCp = yCp+a;
    
end

E = sqrt(Ex.^2 + Ey.^2); % Magnitud del campo eléctrico en cada punto

% Componentes u y v
u = Ex./E; % Componente x del vector unitario del campo eléctrico
v = Ey./E; % Componente y del vector unitario del campo eléctrico

h=quiver(xG,yG,u,v,'autoscalefactor',0.6); % Graficar las flechas en cada punto
set(h,'color',[1 0 0],'linewidth',1.2);
axis([-3.5 3.5 -3.5 3.5]);
axis equal;
box on
hold on

% Cálculo del Campo Eléctrico => Partícula Central 2
magnitudDeCargas = sqrt(Qn^2 + Qp^2); % Magnitud de las cargas
d = sqrt((xCn - xCp)^2 + (yCn - yCp)^2); % Distancia entre las partículas negativa y positiva
p = magnitudDeCargas*d; % Producto de la magnitud de las cargas y la distancia
% Punto medio
xm = (xCn + xCp)/2; % Coordenada x del punto medio
ym = (yCn + yCp)/2; % Coordenada y del punto medio

% Distancia desde el punto medio hasta la partícula central
r = sqrt((xC - xm)^2 + (yC - ym)^2); % Distancia entre el punto medio y la partícula central
vectorPosicionR = (xC - xm) + (yC - ym); % Vector de posición desde el punto medio hasta la partícula central
magnitudDeR = sqrt((xC - xm)^2 + (yC - ym)^2); % Magnitud del vector de posición
vectorUnitarioR = vectorPosicionR/magnitudDeR; % Vector unitario en la dirección del vector de posición
% Campo eléctrico de la partícula central
E = (kC * (2*p/(r^2)) * vectorUnitarioR); % Campo eléctrico en la partícula central

% Graficar las flechas
u_avg = mean(u(:)); % Componente x promedio del vector unitario del campo eléctrico
v_avg = mean(v(:)); % Componente y promedio del vector unitario del campo eléctrico

h=quiver(xC, yC, u_avg*3, v_avg*3, 'Color', 'blue', 'LineWidth', 2, 'MaxHeadSize', 1.5); % Graficar la flecha en la partícula central
set(h,'color',[0 0 1],'linewidth',1.2);
axis([-1.5 1.5 -1.5 1.5]);
axis equal;
box on
hold off

% Campo eléctrico de la partícula central
disp("Coordenadas de la partícula central:"); % Mostrar las coordenadas de la partícula central
disp("x: " + xC);
disp("y: " + yC);
disp("Campo eléctrico de la partícula central:"); % Mostrar el campo eléctrico en la partícula central
disp("E: " + E + "N/C");