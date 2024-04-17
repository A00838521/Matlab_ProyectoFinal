% Clear the workspace, close all figures, and clear the command window
clear; close all; clc;

%Crear grid
N=20; % Number of points in each dimension
minX=-2; maxX=+2; % Minimum and maximum x coordinates
minY=-2; maxY=+2; % Minimum and maximum y coordinates
minZ=-2; maxZ=+2; % Minimum and maximum z coordinates
x=linspace(minX,maxX,N); % Generate a linearly spaced vector of x coordinates
y=linspace(minY,maxY,N); % Generate a linearly spaced vector of y coordinates
z=linspace(minZ,maxZ,N); % Generate a linearly spaced vector of z coordinates
[xG,yG,zG]=meshgrid(x,y,z); % Create a 3D grid of x, y, and z coordinates

% Position of the negative particle
xCn = 0.5; % x coordinate of the negative particle
yCn = 0;   % y coordinate of the negative particle
zCn = 0;   % z coordinate of the negative particle

% Width of the particles
a=0.4;

% Position of the positive particle
xCp = -0.5; % x coordinate of the positive particle
yCp = 1;    % y coordinate of the positive particle
zCp = 0;    % z coordinate of the positive particle

% Position of the central particle
xC = 1; % x coordinate of the central particle
yC = 0; % y coordinate of the central particle
zC = 0; % z coordinate of the central particle

% Electric charges
Qn = 20; % Negative charge of the particle
Qn = abs(Qn) * -1; % Ensure the charge is negative
Qp = 20; % Positive charge of the particle
Qp = abs(Qp) * 1; % Ensure the charge is positive

% Constants
eps0 = 8.85e-12; % Permittivity of free space
kC = 1/(4*pi*eps0); % Coulomb's constant

% 2D plot
Rx = xG - xCn; % x component of the distance between each point and the negative particle
Ry = yG - yCn; % y component of the distance between each point and the negative particle
R = sqrt(Rx.^2 + Ry.^2).^3; % Distance between each point and the negative particle
Ex = kC .* Qn .* Rx ./ R; % x component of the electric field due to the negative particle
Ey = kC .* Qn .* Ry ./ R; % y component of the electric field due to the negative particle
Rx = xG - xCp; % x component of the distance between each point and the positive particle
Ry = yG - yCp; % y component of the distance between each point and the positive particle
R = sqrt(Rx.^2 + Ry.^2).^3; % Distance between each point and the positive particle
Ex = Ex + kC .* Qp .* Rx ./ R; % x component of the electric field due to the positive particle
Ey = Ey + kC .* Qp .* Ry ./ R; % y component of the electric field due to the positive particle
E = sqrt(Ex.^2 + Ey.^2); % Magnitude of the electric field at each point

% Calculate the electric field due to the central particle
magnitudDeCargas = sqrt(Qn^2 + Qp^2); % Magnitude of the charges
d = sqrt((xCn - xCp)^2 + (yCn - yCp)^2 + (zCn - zCp)^2); % Distance between the negative and positive particles
p = magnitudDeCargas*d; % Dipole moment
xm = (xCn + xCp)/2; % x coordinate of the midpoint between the negative and positive particles
ym = (yCn + yCp)/2; % y coordinate of the midpoint between the negative and positive particles
zm = (zCn + zCp)/2; % z coordinate of the midpoint between the negative and positive particles
r = sqrt((xC - xm)^2 + (yC - ym)^2 + (zC - zm)^2); % Distance between the central particle and the midpoint
vectorPosicionR = (xC - xm) + (yC - ym) + (zC - zm); % Position vector from the midpoint to the central particle
magnitudDeR = sqrt((xC - xm)^2 + (yC - ym)^2 + (zC - zm)^2); % Magnitude of the position vector
vectorUnitarioR = vectorPosicionR/magnitudDeR; % Unit vector in the direction of the position vector
E = (kC * (2*p/(r^3)) * vectorUnitarioR); % Electric field due to the central particle

% Plot the electric field vectors
figure();
h=quiver(xG,yG,u,v,'autoscalefactor',0.6); % Plot the electric field vectors
set(h,'color',[1 0 0],'linewidth',1.2); % Set the color and linewidth of the vectors
h=quiver(xC, yC, (xC - xm), (yC - ym), 'Color', 'blue', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Plot the position vector of the central particle
set(h,'color',[1 0 0],'linewidth',1.2); % Set the color and linewidth of the position vector
axis([-1.5 1.5 -1.5 1.5]); % Set the axis limits
axis equal; % Set the aspect ratio of the plot to be equal
box on % Display the box around the plot
h=rectangle('Position',[xCn-a/2,yCn-a/2,a,a],'curvature',[1 1]); % Plot the negative particle
set(h,'Facecolor',[0 0 1],'Edgecolor',[0 0 1]); % Set the color and edge color of the negative particle
text(xCn-0.07,yCn+0.05,'-','Color','white','FontSize',30); % Display the negative sign on the negative particle
h=rectangle('Position',[xCp-a/2,yCp-a/2,a,a],'curvature',[1 1]); % Plot the positive particle
set(h,'Facecolor',[1 0 0],'Edgecolor',[1 0 1]); % Set the color and edge color of the positive particle
text(xCp-0.1,yCp,'+','Color','white','FontSize',30); % Display the positive sign on the positive particle

% Display the coordinates of the central particle and its electric field
disp("Coordenadas de la partícula central:");
disp("x: " + xC);
disp("y: " + yC);
disp("z: " + zC);
disp("Campo eléctrico de la partícula central:");
disp("E: " + E);
%Crear grid
N=20;
minX=-2;maxX=+2;
minY=-2;maxY=+2;
minZ=-2;maxZ=+2;
x=linspace(minX,maxX,N);
y=linspace(minY,maxY,N);
z=linspace(minZ,maxZ,N);
[xG,yG,zG]=meshgrid(x,y,z);
%posicion de la particula negativa
% xCn = input('Ingrese la coordenada x de la primera partícula negativa: ');
% yCn = input('Ingrese la coordenada y de la primera partícula negativa: ');
% zCn = input('Ingrese la coordenada z de la primera partícula negativa: ');
xCn = 0.5;
yCn = 0;
zCn = 0;
a=0.4; % Ancho
%posicion de la particula postitiva
% xCp = input('Ingrese la coordenada x de la primera partícula positiva: ');
% yCp = input('Ingrese la coordenada y de la primera partícula positiva: ');
% zCp = input('Ingrese la coordenada z de la primera partícula positiva: ');
xCp = -0.5;
yCp = 1;
zCp = 0;
% posicion de la particula central
% xC = input('Ingrese la coordenada x de la partícula central: ');
% yC = input('Ingrese la coordenada y de la partícula central: ');
% zC = input('Ingrese la coordenada z de la partícula central: ');
xC = 1;
yC = 0;
zC = 0;

%campo electrico
% Qn = input('Ingrese la carga negativa de la partícula: '); 
% Qn = abs(Qn) * -1;
% Qp = input('Ingrese la carga positiva de la partícula: ');
% Qp = abs(Qp) * 1;
Qn = 20; 
Qn = abs(Qn) * -1;
Qp = 20;
Qp = abs(Qp) * 1;

% Constantes
eps0 = 8.85e-12;
kC = 1/(4*pi*eps0);

% Grafica 2d
Rx = xG - xCn;
Ry = yG - yCn;
R = sqrt(Rx.^2 + Ry.^2).^3;
Ex = kC .* Qn .* Rx ./ R;
Ey = kC .* Qn .* Ry ./ R;
Rx = xG - xCp;
Ry = yG - yCp;
R = sqrt(Rx.^2 + Ry.^2).^3;
Ex = Ex + kC .* Qp .* Rx ./ R;
Ey = Ey + kC .* Qp .* Ry ./ R;
E = sqrt(Ex.^2 + Ey.^2);

%componentes x, y
u = Ex./E;
v = Ey./E;

% Calculo de Campo Electrico => Particula Central 2
magnitudDeCargas = sqrt(Qn^2 + Qp^2);
d = sqrt((xCn - xCp)^2 + (yCn - yCp)^2 + (zCn - zCp)^2);
p = magnitudDeCargas*d;
% Punto medio
xm = (xCn + xCp)/2;
ym = (yCn + yCp)/2;
zm = (zCn + zCp)/2;
% Distancia del punto medio a la particula central
r = sqrt((xC - xm)^2 + (yC - ym)^2 + (zC - zm)^2);
vectorPosicionR = (xC - xm) + (yC - ym) + (zC - zm);
magnitudDeR = sqrt((xC - xm)^2 + (yC - ym)^2 + (zC - zm)^2);
vectorUnitarioR = vectorPosicionR/magnitudDeR;
% Campo electrico de la particula central
E = (kC * (2*p/(r^3)) * vectorUnitarioR);

% Grafica la flecha
figure();

h=quiver(xG,yG,u,v,'autoscalefactor',0.6);
set(h,'color',[1 0 0],'linewidth',1.2);
h=quiver(xC, yC, (xC - xm), (yC - ym), 'Color', 'blue', 'LineWidth', 2, 'MaxHeadSize', 0.5);
set(h,'color',[1 0 0],'linewidth',1.2);
axis([-1.5 1.5 -1.5 1.5]);
axis equal;
box on
h=rectangle('Position',[xCn-a/2,yCn-a/2,a,a],'curvature',[1 1]);
set(h,'Facecolor',[0 0 1],'Edgecolor',[0 0 1]);
text(xCn-0.07,yCn+0.05,'-','Color','white','FontSize',30);
h=rectangle('Position',[xCp-a/2,yCp-a/2,a,a],'curvature',[1 1]);
set(h,'Facecolor',[1 0 0],'Edgecolor',[1 0 1]);
text(xCp-0.1,yCp,'+','Color','white','FontSize',30);



% Campo electrico de la particula central
disp("Coordenadas de la partícula central:");
disp("x: " + xC);
disp("y: " + yC);
disp("z: " + zC);
disp("Campo eléctrico de la partícula central:");
disp("E: " + E);