clear; close all; clc;
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
xCn = input('Ingrese la coordenada x de la primera partícula negativa: ');
yCn = input('Ingrese la coordenada y de la primera partícula negativa: ');
zCn = input('Ingrese la coordenada z de la primera partícula negativa: ');
a=0.4; % Ancho
%posicion de la particula postitiva
xCp = input('Ingrese la coordenada x de la primera partícula positiva: ');
yCp = input('Ingrese la coordenada y de la primera partícula positiva: ');
zCp = input('Ingrese la coordenada z de la primera partícula positiva: ');
%campo electrico
Qn = input('Ingrese la carga negativa de la partícula: '); 
Qn = abs(Qn) * -1;
Qp = input('Ingrese la carga positiva de la partícula: ');
Qp = abs(Qp) * 1;

% Fórmulas
kC = 8.99e+9;
Rx = xG - xCn;
Ry = yG - yCn;
Rz = zG - zCn;
R = sqrt(Rx.^2 + Ry.^2 + Rz.^2).^3;
Ex = kC .* Qn .* Rx ./ R;
Ey = kC .* Qn .* Ry ./ R;
Ez = kC .* Qn .* Rz ./ R;
Rx = xG - xCp;
Ry = yG - yCp;
Rz = zG - zCp;
R = sqrt(Rx.^2 + Ry.^2).^3;
Ex = Ex + kC .* Qp .* Rx ./ R;
Ey = Ey + kC .* Qp .* Ry ./ R;
E = sqrt(Ex.^2 + Ey.^2);
%componentes x, y
u = Ex./E;
v = Ey./E;

figure();
h=quiver(xG,yG,u,v,'autoscalefactor',0.6); % Flechas
set(h,'color',[1 0 0],'linewidth',1.2);
axis([-1.5 1.5 -1.5 1.5]);
axis equal;
box on
h=rectangle('Position',[xCn-a/2,yCn-a/2,a,a],'curvature',[1 1]);
set(h,'Facecolor',[0 0 1],'Edgecolor',[0 0 1]);
text(0.43,0.05,'-','Color','white','FontSize',30);
h=rectangle('Position',[xCp-a/2,yCp-a/2,a,a],'curvature',[1 1]);
set(h,'Facecolor',[1 0 0],'Edgecolor',[1 0 1]);
text(-0.6,0,'+','Color','white','FontSize',30);