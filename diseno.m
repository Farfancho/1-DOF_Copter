%% 1. MODELO IDENTIFICADO
clear; clc; close all;

Ts = 0.02;

K   = 0.14996;
Tp1 = 0.32649;
Tp2 = 0.32733;
Td  = 0.35392;


% Planta continua
G = tf(K, conv([Tp1 1], [Tp2 1]), ...
       'InputDelay', Td);

disp('=== PLANTA CONTINUA ===')
G

% Planta discreta con retencion de orden cero
Gd = c2d(G, Ts, 'zoh');

disp('=== PLANTA DISCRETA ===')
Gd


%% 2. DISENO PI Y PIDF

% Margen de fase objetivo conservador
opts = pidtuneOptions('PhaseMargin', 75);

[C_PI, infoPI] = pidtune(G, 'PI', opts);
[C_PIDF, infoPIDF] = pidtune(G, 'PIDF', opts);

disp('=== PI CONTINUO ===')
C_PI

disp('=== PIDF CONTINUO ===')
C_PIDF


%% 3. DISCRETIZACION POR TUSTIN

C_PI_d = c2d(C_PI, Ts, 'tustin');
C_PIDF_d = c2d(C_PIDF, Ts, 'tustin');

disp('=== PI DISCRETO ===')
tf(C_PI_d)

disp('=== PIDF DISCRETO ===')
tf(C_PIDF_d)


%% 4. COEFICIENTES PARA STM32

cPI = obtenerCoeficientes(C_PI_d);
cPIDF = obtenerCoeficientes(C_PIDF_d);

nombres = {'c1'; 'c2'; 'c3'; 'c4'; 'c5'};

tabla = table(nombres, cPI(:), cPIDF(:), ...
    'VariableNames', {'Coeficiente','PI','PIDF'});

disp('=== COEFICIENTES ECUACION EN DIFERENCIAS ===')
disp(tabla)


%% 5. ESTABILIDAD LINEAL

T_PI = feedback(Gd*C_PI_d, 1);
T_PIDF = feedback(Gd*C_PIDF_d, 1);

fprintf('\nPI estable: %d\n', isstable(T_PI));
fprintf('PIDF estable: %d\n', isstable(T_PIDF));

fprintf('Margen fase PI: %.2f grados\n', ...
    infoPI.PhaseMargin);

fprintf('Margen fase PIDF: %.2f grados\n', ...
    infoPIDF.PhaseMargin);


%% 6. SIMULACION CON SATURACION PWM

PWMmin = 0;
PWMmax = 999;

% Punto de operacion estimado
PWMeq = 318;
anguloEq = 88;

% Cambio pequeno alrededor del equilibrio
referencia = 90;

t = (0:Ts:12)';

[yPI, uPI, satPI] = simularControl( ...
    Gd, cPI, t, referencia, anguloEq, ...
    PWMeq, PWMmin, PWMmax);

[yPIDF, uPIDF, satPIDF] = simularControl( ...
    Gd, cPIDF, t, referencia, anguloEq, ...
    PWMeq, PWMmin, PWMmax);


%% 7. GRAFICAS

figure;

subplot(2,1,1)

plot(t, yPI, 'LineWidth', 1.5)
hold on
plot(t, yPIDF, 'LineWidth', 1.5)
yline(referencia, 'k--')

grid on
xlabel('Tiempo (s)')
ylabel('Angulo (grados)')
title('Respuesta angular')
legend('PI', 'PIDF', 'Referencia')


subplot(2,1,2)

plot(t, uPI, 'LineWidth', 1.5)
hold on
plot(t, uPIDF, 'LineWidth', 1.5)
yline(PWMeq, 'k--')
yline(PWMmin, 'k:')
yline(PWMmax, 'k:')

grid on
xlabel('Tiempo (s)')
ylabel('PWM (0-999)')
title('Esfuerzo de control')
legend('PI', 'PIDF', 'PWM equilibrio')

fprintf('\nMuestras saturadas PI: %d\n', sum(satPI));
fprintf('Muestras saturadas PIDF: %d\n', sum(satPIDF));


%% 8. FUNCIONES LOCALES

function c = obtenerCoeficientes(Cd)

    [b,a] = tfdata(tf(Cd), 'v');

    b = b/a(1);
    a = a/a(1);

    % Alinear grados
    n = max(numel(a), numel(b));

    if n > 3
        error('El controlador supera el orden 2');
    end

    b = [zeros(1,n-numel(b)), b];
    a = [zeros(1,n-numel(a)), a];

    % Completar a segundo orden
    b = [b, zeros(1,3-n)];
    a = [a, zeros(1,3-n)];

    % Ecuacion:
    % u[k] = c1*u[k-1] + c2*u[k-2]
    %      + c3*e[k] + c4*e[k-1] + c5*e[k-2]

    c = [-a(2), -a(3), b(1), b(2), b(3)];

end


function [angulo,pwm,saturado] = simularControl( ...
    Gd,c,t,referencia,anguloEq,PWMeq,PWMmin,PWMmax)

    % Incorporar el retardo discreto a los estados
    planta = absorbDelay(ss(Gd));
    [A,B,C,D] = ssdata(planta);

    if norm(D,inf) > 1e-9
        error('La planta tiene paso directo');
    end

    N = numel(t);
    x = zeros(size(A,1),1);

    angulo = zeros(N,1);
    pwm = zeros(N,1);
    saturado = false(N,1);

    e1 = 0;
    e2 = 0;
    u1 = 0;
    u2 = 0;

    for k = 1:N

        % Salida de la planta
        angulo(k) = anguloEq + C*x;

        % Error actual
        e = referencia - angulo(k);

        % Controlador discreto incremental
        uRaw = c(1)*u1 + c(2)*u2 + ...
               c(3)*e + c(4)*e1 + c(5)*e2;

        % Saturacion relativa al PWM de equilibrio
        du = min(max(uRaw, PWMmin-PWMeq), ...
                 PWMmax-PWMeq);

        saturado(k) = abs(du-uRaw) > 1e-9;

        % PWM absoluto
        pwm(k) = PWMeq + du;

        % Actualizar planta
        x = A*x + B*du;

        % Actualizar historiales
        e2 = e1;
        e1 = e;

        u2 = u1;
        u1 = du;

    end
end