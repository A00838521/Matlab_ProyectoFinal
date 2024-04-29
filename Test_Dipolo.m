% Crear grid
clc; clear; close all; % Limpiar la consola, las variables y las figuras
N=20; % Número de puntos en cada dimensión
minX=-2; maxX=+2; % Valores mínimos y máximos de x
minY=-2; maxY=+2; % Valores mínimos y máximos de y
x=linspace(minX,maxX,N); % Generar valores de x
y=linspace(minY,maxY,N); % Generar valores de y
[xG,yG]=meshgrid(x,y); % Crear una cuadrícula de puntos en el espacio 2D

direccion = input("Ingrese la dirección de las cargas (1: Horizontal, 2: Vertical): "); % Dirección de las cargas

% Información de las partículas
NQn = 6; % Número de partículas negativas
NQp = 3; % Número de partículas positivas

% Información de carga negativa
xCn = 0.5; % Coordenada x de la partícula negativa
yCn = -1; % Coordenada y de la partícula negativa
Qn = -20; % Carga negativa de la partícula

% Posición de la partícula positiva
xCp =-0.5; % Coordenada x de la partícula positiva
yCp =-0.5; % Coordenada y de la partícula positiva
Qp = 20; % Carga positiva de la partícula

% Ancho de las partículas
a=0.4; % Ancho de la partícula

% Constantes
eps0 = 8.85e-12; % Permitividad del espacio libre
kC = 1/(4*pi*eps0); % Constante de Coulomb

u = zeros(size(xG)); % Componente x del campo eléctrico
v = zeros(size(yG)); % Componente y del campo eléctrico

% Graficar la partícula negativa
if direccion == 1
    for i=1:NQn
        for j=1:NQn
            Rx = xG - xCn - (a * (j-1)); % Componente x de la distancia entre cada punto y la partícula negativa
            Ry = yG - yCn; % Componente y de la distancia entre cada punto y la partícula negativa
            R = sqrt(Rx.^2 + Ry.^2).^3; % Distancia entre cada punto y la partícula negativa
            Ex = kC .* Qn .* Rx ./ R; % Componente x del campo eléctrico debido a la partícula negativa
            Ey = kC .* Qn .* Ry ./ R; % Componente y del campo eléctrico debido a la partícula negativa
            
            u = u + Ex; % Sumar las componentes x del campo eléctrico
            v = v + Ey; % Sumar las componentes y del campo eléctrico
        end
    end

    % Graficar la partícula positiva
    for i=1:NQp
        for j=1:NQp
            Rx = xG - xCp - (a * (j-1)); % Componente x de la distancia entre cada punto y la partícula positiva
            Ry = yG - yCp; % Componente y de la distancia entre cada punto y la partícula positiva
            R = sqrt(Rx.^2 + Ry.^2).^3; % Distancia entre cada punto y la partícula positiva
            Ex = kC .* Qp .* Rx ./ R; % Componente x del campo eléctrico debido a la partícula positiva
            Ey = kC .* Qp .* Ry ./ R; % Componente y del campo eléctrico debido a la partícula positiva
            u = u + Ex; % Sumar las componentes x del campo eléctrico
            v = v + Ey; % Sumar las componentes y del campo eléctrico
        end
    end
elseif direccion == 2
    for i=1:NQn
        for j=1:NQn
            Rx = xG - xCn; % Componente x de la distancia entre cada punto y la partícula negativa
            Ry = yG - yCn - (a * (j-1)); % Componente y de la distancia entre cada punto y la partícula negativa
            R = sqrt(Rx.^2 + Ry.^2).^3; % Distancia entre cada punto y la partícula negativa
            Ex = kC .* Qn .* Rx ./ R; % Componente x del campo eléctrico debido a la partícula negativa
            Ey = kC .* Qn .* Ry ./ R; % Componente y del campo eléctrico debido a la partícula negativa
            
            u = u + Ex; % Sumar las componentes x del campo eléctrico
            v = v + Ey; % Sumar las componentes y del campo eléctrico
        end
    end

    % Graficar la partícula positiva
    for i=1:NQp
        for j=1:NQp
            Rx = xG - xCp; % Componente x de la distancia entre cada punto y la partícula positiva
            Ry = yG - yCp - (a * (j-1)); % Componente y de la distancia entre cada punto y la partícula positiva
            R = sqrt(Rx.^2 + Ry.^2).^3; % Distancia entre cada punto y la partícula positiva
            Ex = kC .* Qp .* Rx ./ R; % Componente x del campo eléctrico debido a la partícula positiva
            Ey = kC .* Qp .* Ry ./ R; % Componente y del campo eléctrico debido a la partícula positiva
            u = u + Ex; % Sumar las componentes x del campo eléctrico
            v = v + Ey; % Sumar las componentes y del campo eléctrico
        end
    end
end

E = sqrt(u.^2 + v.^2); % Magnitud del campo eléctrico en cada punto

% Componentes u y v
u = u./E; % Componente x del vector unitario del campo eléctrico
v = v./E; % Componente y del vector unitario del campo eléctrico

% Graficar las flechas
u_avg = mean(u(:)); % Componente x promedio del vector unitario del campo eléctrico
v_avg = mean(v(:)); % Componente y promedio del vector unitario del campo eléctrico

figure();
quiver(xG, yG, u, v, 'autoscalefactor', 0.6, 'color', [1 0 0], 'linewidth', 1.2); % Graficar las flechas del campo eléctrico
hold on;

if direccion == 1
    for i=1:NQn
        for j=1:NQn
            plot(xCn + a * (0:NQn-1), yCn * ones(1, NQn), 'bo', 'MarkerSize', 15, 'MarkerFaceColor', 'b'); % Cargas negativas
            plot(xCp + a * (0:NQp-1), yCp * ones(1, NQp), 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r'); % Cargas positivas
        end
    end
elseif direccion == 2
    for i=1:NQn
        for j=1:NQn
            plot(xCn * ones(1, NQn) , yCn + a * (0:NQn-1), 'bo', 'MarkerSize', 15, 'MarkerFaceColor', 'b'); % Cargas negativas
            plot(xCp * ones(1, NQp), yCp + a * (0:NQp-1), 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r'); % Cargas positivas
        end
    end
end
axis([-1.5 1.5 -1.5 1.5]); % Establecer los límites del eje x y el eje y
axis equal; % Establecer la relación de aspecto igual
box on; % Mostrar el borde de la figura

%%%

% Posición de la partícula central
xC = -1; % Coordenada x de la partícula central
yC = 1; % Coordenada y de la partícula central
xC2 = 1; % Coordenada y de la partícula central
yC2 = -1; % Coordenada y de la partícula central
aParticula=0.1; % Ancho de la partícula central
plot(xC, yC, 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'y'); % Partícula central
plot(xC2, yC2, 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'g'); % Partícula central

% Cálculo del Campo Eléctrico => Partícula Central 2
magnitudDeCargas = sqrt(Qn^2 + Qp^2); % Magnitud de las cargas
d = sqrt((xCn - xCp)^2 + (yCn - yCp)^2); % Distancia entre las partículas negativa y positiva
p = magnitudDeCargas*d; % Producto de la magnitud de las cargas y la distancia

% Punto medio
xm = 0; % Coordenada x del punto medio
ym = 0; % Coordenada y del punto medio
for i=1:NQn
    for j=1:NQn
        xm =(xCn + xCp + xm)/2; % Coordenada x del punto medio
        ym =(yCn + yCp + ym)/2; % Coordenada y del punto medio
        r = sqrt((xC - xm)^2 + (yC - ym)^2); % Distancia entre el punto medio y la partícula central
        vectorPosicionR = (xC - xm) + (yC - ym); % Vector de posición desde el punto medio hasta la partícula central
        magnitudDeR = sqrt((xC - xm)^2 + (yC - ym)^2); % Magnitud del vector de posición
        vectorUnitarioR = vectorPosicionR/magnitudDeR; % Vector unitario en la dirección del vector de posición
        E = E + (kC * (2*p/(r^2)) * vectorUnitarioR); % Campo eléctrico en la partícula central
        
    end
end

% Distancia desde el punto medio hasta la partícula central
r2 = sqrt((xC2 - xm)^2 + (yC2 - ym)^2); % Distancia entre el punto medio y la partícula central
vectorPosicionR2 = (xC2 - xm) + (yC2 - ym); % Vector de posición desde el punto medio hasta la partícula central
magnitudDeR2 = sqrt((xC2 - xm)^2 + (yC2 - ym)^2); % Magnitud del vector de posición
vectorUnitarioR2 = vectorPosicionR2/magnitudDeR2; % Vector unitario en la dirección del vector de posición

% Campo eléctrico de la partícula central
E2 = (kC * (2*p/(r2^2)) * vectorUnitarioR2); % Campo eléctrico en la partícula central

u = u./vectorUnitarioR;
v = u./vectorUnitarioR;

U2 = u./vectorUnitarioR2;
V2 = u./vectorUnitarioR2;

% Graficar las flechas
posU = u(1); % Componente x del campo eléctrico
posV = v(1); % Componente y del campo eléctrico
posU2 = U2(1); % Componente x del campo eléctrico
posV2 = V2(1); % Componente y del campo eléctrico
h=quiver(xC, yC, posU, posV, 'Color', 'blue', 'LineWidth', 2, 'MaxHeadSize', 1.5); % Graficar la flecha en la partícula central
set(h,'color',[0 1 0],'linewidth',1.2);
axis equal;
hold on;
h=quiver(xC2, yC2, posU2, posV2, 'Color', 'blue', 'LineWidth', 2, 'MaxHeadSize', 1.5); % Graficar la flecha en la partícula central
set(h,'color',[0 0 1],'linewidth',1.2);
axis equal;
box on
hold off;

% Campo eléctrico de la partícula central
disp("Coordenadas de la partícula central:"); % Mostrar las coordenadas de la partícula central
disp("x: " + xC);
disp("y: " + yC);
disp("Campo eléctrico de la partícula central 1 color amarillo:"); % Mostrar el campo eléctrico en la partícula central
disp("E in N/C: ");
format shortE
disp(E)
disp("Coordenadas de la partícula central 2:"); % Mostrar las coordenadas de la partícula central
disp("x: " + xC2);
disp("y: " + yC2);
disp("Campo eléctrico de la partícula central 2 color verde:"); % Mostrar el campo eléctrico en la partícula central
disp("E in N/C: ");
format shortE
disp(E2)