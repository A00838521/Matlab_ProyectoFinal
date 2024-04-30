% Importar clases
import NegativeCharge.*; % Importar todas las clases del paquete NegativeCharge
import PositiveCharge.*; % Importar todas las clases del paquete PositiveCharge
import NeutralParticle.*; % Importar todas las clases del paquete NeutralParticle
import matlab.graphics.function.quiver.*; % Importar la clase quiver del paquete matlab.graphics.function.quiver

% Número de puntos en cada dimensión y configuración de la cuadrícula
N = 20; % Número de puntos en cada dimensión
minX = -2; maxX = 2; % Límites en el eje x
minY = -2; maxY = 2; % Límites en el eje y
x = linspace(minX, maxX, N); % Generar un vector de N puntos equiespaciados en el rango [minX, maxX]
y = linspace(minY, maxY, N); % Generar un vector de N puntos equiespaciados en el rango [minY, maxY]
[xG, yG] = meshgrid(x, y); % Generar una cuadrícula de puntos a partir de los vectores x e y

% Numero de particulas negativas y positivas
NQn = input("Ingrese el número de partículas negativas: "); % Solicitar al usuario el número de partículas negativas
NQp = input("Ingrese el número de partículas positivas: "); % Solicitar al usuario el número de partículas positivas

% Posiciones de las partículas
inXp = input("Ingrese la coordenada x de la primera partícula positiva: "); % Solicitar al usuario la coordenada x de la primera partícula positiva
inYp = input("Ingrese la coordenada y de la primera partícula positiva: "); % Solicitar al usuario la coordenada y de la primera partícula positiva
inXn = input("Ingrese la coordenada x de la primera partícula negativa: "); % Solicitar al usuario la coordenada x de la primera partícula negativa
inYn = input("Ingrese la coordenada y de la primera partícula negativa: "); % Solicitar al usuario la coordenada y de la primera partícula negativa

a = 0.4; % Ancho de las partículas

% Creación de partículas usando las clases
negativeCharges = [NegativeCharge(inXn, inYn)]; % Crear un arreglo de partículas negativas con la primera partícula
positiveCharges = [PositiveCharge(inXp, inYp)]; % Crear un arreglo de partículas positivas con la primera partícula
direccion = input("Ingrese la dirección de las cargas (1: Horizontal, 2: Vertical): "); % Solicitar al usuario la dirección de las cargas

if direccion == 1
    for i=1:NQn
        negativeCharges = [negativeCharges, NegativeCharge(inXn, inYn)]; %#ok<AGROW> % Agregar una partícula negativa al arreglo en la dirección horizontal
        inXn = inXn + a; % Incrementar la coordenada x para la siguiente partícula negativa
    end

    % Graficar la partícula positiva
    for i=1:NQp
        positiveCharges = [positiveCharges, PositiveCharge(inXp, inYp)]; %#ok<AGROW> % Agregar una partícula positiva al arreglo en la dirección horizontal
        inXp = inXp + a; % Incrementar la coordenada x para la siguiente partícula positiva
    end
elseif direccion == 2
    for i=1:NQn
        negativeCharges = [negativeCharges, NegativeCharge(inXn, inYn)]; %#ok<AGROW> % Agregar una partícula negativa al arreglo en la dirección vertical
        inYn = inYn + a; % Incrementar la coordenada y para la siguiente partícula negativa
    end

    % Graficar la partícula positiva
    for i=1:NQp
        positiveCharges = [positiveCharges, PositiveCharge(inXp, inYp)]; %#ok<AGROW> % Agregar una partícula positiva al arreglo en la dirección vertical
        inYp = inYp + a; % Incrementar la coordenada y para la siguiente partícula positiva
    end    
end    


% Entrada de posiciones de las partículas neutras
xN1 = input("Ingrese la coordenada x de la primera partícula neutral: "); % Solicitar al usuario la coordenada x de la primera partícula neutral
yN1 = input("Ingrese la coordenada y de la primera partícula neutral: "); % Solicitar al usuario la coordenada y de la primera partícula neutral
xN2 = input("Ingrese la coordenada x de la segunda partícula neutral: "); % Solicitar al usuario la coordenada x de la segunda partícula neutral
yN2 = input("Ingrese la coordenada y de la segunda partícula neutral: "); % Solicitar al usuario la coordenada y de la segunda partícula neutral

neutralParticles = [NeutralParticle(xN1, yN1), NeutralParticle(xN2, yN2)]; % Crear un arreglo de partículas neutras con las coordenadas ingresadas

% Constantes
eps0 = 8.85e-12; % Constante de permitividad eléctrica del vacío
kC = 1/(4*pi*eps0); % Constante de Coulomb

% Cálculo del campo eléctrico
u = zeros(size(xG)); % Inicializar el vector u con ceros
v = zeros(size(yG)); % Inicializar el vector v con ceros

charges = {negativeCharges, positiveCharges}; % Arreglo que contiene los arreglos de partículas negativas y positivas

for i = 1:length(charges)
    chargeArray = charges{i}; % Obtener el arreglo de partículas correspondiente
    for j = 1:length(chargeArray)
        charge = chargeArray(j); % Obtener la partícula correspondiente
        Rx = xG - charge.x; % Distancia en el eje x entre los puntos de la cuadrícula y la partícula
        Ry = yG - charge.y; % Distancia en el eje y entre los puntos de la cuadrícula y la partícula
        R = sqrt(Rx.^2 + Ry.^2).^3; % Distancia al cubo entre los puntos de la cuadrícula y la partícula
        u = u + kC * charge.Q .* Rx ./ R; % Componente x del campo eléctrico debido a la partícula
        v = v + kC * charge.Q .* Ry ./ R; % Componente y del campo eléctrico debido a la partícula
    end
end

% Normalización de los vectores del campo eléctrico
E = sqrt(u.^2 + v.^2); % Magnitud del campo eléctrico en cada punto de la cuadrícula
u = u ./ E; % Componente x normalizada del campo eléctrico
v = v ./ E; % Componente y normalizada del campo eléctrico

% Graficar campo eléctrico
figure(); % Crear una nueva figura
quiver(xG, yG, u, v, 'autoscalefactor', 0.6, 'color', [1 0 0], 'linewidth', 1.2); % Graficar el campo eléctrico usando flechas en la cuadrícula
hold on; % Mantener la figura actual para agregar más elementos

% Graficar partículas
arrayfun(@(c) c.draw(), negativeCharges); % Graficar todas las partículas negativas
arrayfun(@(c) c.draw(), positiveCharges); % Graficar todas las partículas positivas
arrayfun(@(c) c.draw(), neutralParticles); % Graficar todas las partículas neutras

% Calcular el campo eléctrico en las posiciones de las partículas neutras y la flecha hacia la carga negativa más cercana
for neutral = neutralParticles
    uNeut = 0; % Componente x del campo eléctrico en la posición de la partícula neutra
    vNeut = 0; % Componente y del campo eléctrico en la posición de la partícula neutra
    for i = 1:length(charges)
        chargeArray = charges{i}; % Obtener el arreglo de partículas correspondiente
        for j = 1:length(chargeArray)
            charge = chargeArray(j); % Obtener la partícula correspondiente
            Rx = neutral.x - charge.x; % Distancia en el eje x entre la partícula neutra y la partícula cargada
            Ry = neutral.y - charge.y; % Distancia en el eje y entre la partícula neutra y la partícula cargada
            R = sqrt(Rx.^2 + Ry.^2).^3; % Distancia al cubo entre la partícula neutra y la partícula cargada
            uNeut = uNeut + kC * charge.Q * Rx / R; % Componente x del campo eléctrico en la posición de la partícula neutra debido a la partícula cargada
            vNeut = vNeut + kC * charge.Q * Ry / R; % Componente y del campo eléctrico en la posición de la partícula neutra debido a la partícula cargada
        end
    end
    % quiver(neutral.x, neutral.y, uNeut, vNeut, 'MaxHeadSize', 2, 'Color', 'm', 'LineWidth', 2);

    % Encontrar la carga negativa más cercana
    distances = arrayfun(@(n) sqrt((neutral.x - n.x)^2 + (neutral.y - n.y)^2), negativeCharges); % Calcular las distancias entre la partícula neutra y todas las partículas negativas
    [~, idx] = min(distances); % Obtener el índice de la carga negativa más cercana
    closestNeg = negativeCharges(idx); % Obtener la carga negativa más cercana
    quiver(neutral.x, neutral.y, closestNeg.x - neutral.x, closestNeg.y - neutral.y, 'MaxHeadSize', 2, 'Color', 'green', 'LineWidth', 2, 'LineStyle', '-'); % Graficar una flecha desde la partícula neutra hacia la carga negativa más cercana
end

% Ajustes de los ejes
axis([-2 2 -2 2]); % Establecer los límites de los ejes
axis equal; % Establecer la relación de aspecto igual para los ejes
box on; % Mostrar los ejes y la caja del gráfico
