% Ingresar las cargas de las partículas
carga1 = input('Ingrese la carga de la primera partícula en Coulombs: ');
carga2 = input('Ingrese la carga de la segunda partícula en Coulombs: ');
carga3 = input('Ingrese la carga de la tercera partícula en Coulombs: ');

% Ingresar las coordenadas de las partículas
x1 = input('Ingrese la coordenada x de la primera partícula: ');
y1 = input('Ingrese la coordenada y de la primera partícula: ');
z1 = input('Ingrese la coordenada z de la primera partícula: ');

x2 = input('Ingrese la coordenada x de la segunda partícula: ');
y2 = input('Ingrese la coordenada y de la segunda partícula: ');
z2 = input('Ingrese la coordenada z de la segunda partícula: ');

% Ingresar las coordenadas de la tercera partícula
x3 = input('Ingrese la coordenada x de la tercera partícula: ');
y3 = input('Ingrese la coordenada y de la tercera partícula: ');
z3 = input('Ingrese la coordenada z de la tercera partícula: ');

% Crear una figura y un gráfico 3D
figure;
h = plot3([x1 x2], [y1 y2], [z1 z2], 'ro');
hold on;
grid on;
axis equal;

% Calcular la distancia entre las partículas
distancia = sqrt((x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2);

% Calcular la fuerza entre las partículas usando la ley de Coulomb
k = 8.99e9; % Constante de Coulomb
fuerza = k * abs(carga1 * carga2) / distancia^2;

% Calcular la distancia entre la tercera partícula y las otras dos partículas
distancia3_1 = sqrt((x3 - x1)^2 + (y3 - y1)^2 + (z3 - z1)^2);
distancia3_2 = sqrt((x3 - x2)^2 + (y3 - y2)^2 + (z3 - z2)^2);

% Calcular la fuerza entre la tercera partícula y las otras dos partículas usando la ley de Coulomb
fuerza3_1 = k * abs(carga1 * carga3) / distancia3_1^2;
fuerza3_2 = k * abs(carga2 * carga3) / distancia3_2^2;

% Calcular el campo eléctrico de la tercera partícula
campo_electrico3 = fuerza3_1 / carga3;

% Calcular la dirección del campo eléctrico de la tercera partícula
direccion_x3 = (x3 - x1) / distancia3_1;
direccion_y3 = (y3 - y1) / distancia3_1;
direccion_z3 = (z3 - z1) / distancia3_1;

% Mostrar el resultado
fprintf('El campo eléctrico de la tercera partícula es: %.2f N/C en la dirección (%.2f, %.2f, %.2f)\n', campo_electrico3, direccion_x3, direccion_y3, direccion_z3);

% Callback para el movimiento del mouse
set(gcf, 'WindowButtonMotionFcn', @dragObject);

% Callback para el clic del mouse
set(gcf, 'WindowButtonDownFcn', @startDrag);

% Variables para el arrastre
dragging = false;
offset = [0 0 0];

% Función para iniciar el arrastre
function startDrag(~,~)
    dragging = true;
    cp = get(gca,'CurrentPoint');
    offset = cp(1,1:3) - [x1 y1 z1];
end

% Función para arrastrar el objeto
function dragObject(~,~)
    if dragging
        cp = get(gca,'CurrentPoint');
        x1 = cp(1,1) - offset(1);
        y1 = cp(1,2) - offset(2);
        z1 = cp(1,3) - offset(3);
        set(h, 'XData', [x1 x2], 'YData', [y1 y2], 'ZData', [z1 z2]);
        drawnow;
    end
end