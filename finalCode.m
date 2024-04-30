% Importar clases
import NegativeCharge.*;
import PositiveCharge.*;
import NeutralParticle.*;
import matlab.graphics.function.quiver.*; % Add this line

% Número de puntos en cada dimensión y configuración de la cuadrícula
N = 20;
minX = -2; maxX = 2;
minY = -2; maxY = 2;
x = linspace(minX, maxX, N);
y = linspace(minY, maxY, N);
[xG, yG] = meshgrid(x, y);

% Numero de particulas negativas y positivas
NQn = input("Ingrese el número de partículas negativas: ");
NQp = input("Ingrese el número de partículas positivas: ");

% Posiciones de las partículas
inXp = input("Ingrese la coordenada x de la primera partícula positiva: ");
inYp = input("Ingrese la coordenada y de la primera partícula positiva: ");
inXn = input("Ingrese la coordenada x de la primera partícula negativa: ");
inYn = input("Ingrese la coordenada y de la primera partícula negativa: ");

a = 0.4; % Ancho de las partículas
    
% Creación de partículas usando las clases
negativeCharges = [NegativeCharge(inXn, inYn)];
positiveCharges = [PositiveCharge(inXp, inYp)];
direccion = input("Ingrese la dirección de las cargas (1: Horizontal, 2: Vertical): "); % Dirección de las cargas

if direccion == 1
    for i=1:NQn
        negativeCharges = [negativeCharges, NegativeCharge(inXn, inYn)]; %#ok<AGROW>
        inXn = inXn + a;
    end

    % Graficar la partícula positiva
    for i=1:NQp
        positiveCharges = [positiveCharges, PositiveCharge(inXp, inYp)]; %#ok<AGROW>
        inXp = inXp + a;
    end
elseif direccion == 2
    for i=1:NQn
        negativeCharges = [negativeCharges, NegativeCharge(inXn, inYn)]; %#ok<AGROW>
        inYn = inYn + a;
    end

    % Graficar la partícula positiva
    for i=1:NQp
        positiveCharges = [positiveCharges, PositiveCharge(inXp, inYp)]; %#ok<AGROW> 
        inYp = inYp + a;
    end    
end    


% Entrada de posiciones de las partículas neutras
xN1 = input("Ingrese la coordenada x de la primera partícula neutral: ");
yN1 = input("Ingrese la coordenada y de la primera partícula neutral: ");
xN2 = input("Ingrese la coordenada x de la segunda partícula neutral: ");
yN2 = input("Ingrese la coordenada y de la segunda partícula neutral: ");

neutralParticles = [NeutralParticle(xN1, yN1), NeutralParticle(xN2, yN2)];

% Constantes
eps0 = 8.85e-12; 
kC = 1/(4*pi*eps0);

% Cálculo del campo eléctrico
u = zeros(size(xG)); 
v = zeros(size(yG));

charges = {negativeCharges, positiveCharges};

for i = 1:length(charges)
    chargeArray = charges{i};
    for j = 1:length(chargeArray)
        charge = chargeArray(j);
        Rx = xG - charge.x;
        Ry = yG - charge.y;
        R = sqrt(Rx.^2 + Ry.^2).^3;
        u = u + kC * charge.Q .* Rx ./ R; 
        v = v + kC * charge.Q .* Ry ./ R;
    end
end

% Normalización de los vectores del campo eléctrico
E = sqrt(u.^2 + v.^2);
u = u ./ E;
v = v ./ E;

% Graficar campo eléctrico
figure();
quiver(xG, yG, u, v, 'autoscalefactor', 0.6, 'color', [1 0 0], 'linewidth', 1.2);
hold on;

% Graficar partículas
arrayfun(@(c) c.draw(), negativeCharges);
arrayfun(@(c) c.draw(), positiveCharges);
arrayfun(@(c) c.draw(), neutralParticles);

% Calcular el campo eléctrico en las posiciones de las partículas neutras y la flecha hacia la carga negativa más cercana
for neutral = neutralParticles
    uNeut = 0;
    vNeut = 0;
    for i = 1:length(charges)
        chargeArray = charges{i};
        for j = 1:length(chargeArray)
            charge = chargeArray(j);
            Rx = neutral.x - charge.x;
            Ry = neutral.y - charge.y;
            R = sqrt(Rx.^2 + Ry.^2).^3;
            uNeut = uNeut + kC * charge.Q * Rx / R;
            vNeut = vNeut + kC * charge.Q * Ry / R;
        end
    end
    % quiver(neutral.x, neutral.y, uNeut, vNeut, 'MaxHeadSize', 2, 'Color', 'm', 'LineWidth', 2);

    % Encontrar la carga negativa más cercana
    distances = arrayfun(@(n) sqrt((neutral.x - n.x)^2 + (neutral.y - n.y)^2), negativeCharges);
    [~, idx] = min(distances);
    closestNeg = negativeCharges(idx);
    quiver(neutral.x, neutral.y, closestNeg.x - neutral.x, closestNeg.y - neutral.y, 'MaxHeadSize', 2, 'Color', 'green', 'LineWidth', 2, 'LineStyle', '-');
end

% Ajustes de los ejes
axis([-2 2 -2 2]);
axis equal;
box on;
