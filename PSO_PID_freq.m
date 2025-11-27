function [Kp_best, Ki_best, Kd_best, gbest_cost] = PSO_PID_Simulink()
    clc; clear; close all;
    
    % --- Configuración ---
    model_name = 'PSO_PID_Frecuencia'; % <--- PON AQUÍ EL NOMBRE EXACTO (sin .slx)
    num_particles = 5;
    max_iter = 5;
    
    % Límites de búsqueda [Kc, Ti, Td]
    VarMin = [0.001   0.001  0.0];  % Bajamos el mínimo de Kc
    VarMax = [50.0    5.0    2.0];  % Subimos el máximo de Kc
    nVar = 3;

    % Parámetros PSO
    w = 0.7; c1 = 1.5; c2 = 1.5;

    % Inicialización
    empty_particle.pos = [];
    empty_particle.vel = [];
    empty_particle.cost = [];
    empty_particle.best.pos = [];
    empty_particle.best.cost = [];
    
    particle = repmat(empty_particle, num_particles, 1);
    GlobalBest.cost = inf;
    GlobalBest.pos = [];
    
    % --- Bucle Inicial ---
    fprintf('Inicializando partículas...\n');
    for i = 1:num_particles
        particle(i).pos = VarMin + rand(1,nVar).*(VarMax - VarMin);
        particle(i).vel = zeros(1,nVar);
        
        % Evaluar costo llamando a Simulink
        particle(i).cost = cost_function_slx(particle(i).pos, model_name);
        
        particle(i).best.pos = particle(i).pos;
        particle(i).best.cost = particle(i).cost;
        
        if particle(i).best.cost < GlobalBest.cost
            GlobalBest = particle(i).best;
        end
    end
    
    % --- Bucle Principal ---
    for it = 1:max_iter
        for i = 1:num_particles
            % Actualizar Velocidad y Posición
            r1 = rand(1,nVar); r2 = rand(1,nVar);
            particle(i).vel = w*particle(i).vel ...
                + c1*r1.*(particle(i).best.pos - particle(i).pos) ...
                + c2*r2.*(GlobalBest.pos - particle(i).pos);
            
            particle(i).pos = particle(i).pos + particle(i).vel;
            
            % Restringir límites
            particle(i).pos = max(particle(i).pos, VarMin);
            particle(i).pos = min(particle(i).pos, VarMax);
            
            % Evaluar Costo
            particle(i).cost = cost_function_slx(particle(i).pos, model_name);
            
            % Actualizar PBest y GBest
            if particle(i).cost < particle(i).best.cost
                particle(i).best.pos = particle(i).pos;
                particle(i).best.cost = particle(i).cost;
                if particle(i).best.cost < GlobalBest.cost
                    GlobalBest = particle(i).best;
                end
            end
        end
        fprintf('Iteración %d | Mejor Costo: %.4f | Kc: %.4f\n', it, GlobalBest.cost, GlobalBest.pos(1));
    end
    
    % Resultados
    Kp_best = GlobalBest.pos(1);
    Ki_best = GlobalBest.pos(2);
    Kd_best = GlobalBest.pos(3);
    
    % Asignar los mejores valores al workspace para que puedas correr la simulación final manual
    assignin('base', 'Kc_sim', Kp_best);
    assignin('base', 'Ti_sim', Ki_best);
    assignin('base', 'Td_sim', Kd_best);
    
    disp('Optimización Completa.');
    disp(['Mejor Kc: ', num2str(Kp_best)]);
    disp(['Mejor Ti: ', num2str(Ki_best)]);
    disp(['Mejor Td: ', num2str(Kd_best)]);
end

function cost = cost_function_slx(x, model_name)
    % 1. Configuración de la simulación
    simIn = Simulink.SimulationInput(model_name);
    simIn = simIn.setVariable('Kc_sim', x(1));
    simIn = simIn.setVariable('Ti_sim', x(2));
    simIn = simIn.setVariable('Td_sim', x(3));

    try
        % 2. Ejecutar Simulación
        out = sim(simIn);
        
        % 3. BÚSQUEDA INTELIGENTE DE LA VARIABLE (La parte importante)
        e = []; % Inicializamos vacío
        
        % Opción A: Está directo como propiedad (Formato Array/Timeseries)
        if isprop(out, 'error_sim')
            raw = out.error_sim;
            if isa(raw, 'timeseries'), e = raw.Data; else, e = raw; end
            
        % Opción B: Está accesible vía método get()
        elseif isa(out, 'Simulink.SimulationOutput') && ~isempty(out.find('error_sim'))
            raw = out.get('error_sim');
            if isa(raw, 'timeseries'), e = raw.Data; else, e = raw; end
            
        % Opción C: Está escondido en 'logsout' (Formato Dataset - MUY COMÚN)
        elseif isprop(out, 'logsout') && ~isempty(out.logsout.find('error_sim'))
            % En Dataset, hay que extraer .Values y luego .Data
            e = out.logsout.get('error_sim').Values.Data;
        end
        
        % 4. Verificación final
        if isempty(e)
            % Si llegamos aquí, imprimimos qué hay en 'out' para depurar
            disp('--- ERROR DE LECTURA ---');
            disp('Variables encontradas en out:');
            disp(out.who);
            if isprop(out, 'logsout'), disp(out.logsout); end
            cost = 1e10; 
            return;
        end

        % 5. Cálculo del Costo
        if any(isnan(e)) || any(isinf(e))
            cost = 1e10;
        else
            cost = mean(e.^2);
        end

    catch ME
        % Si falla la simulación en sí
        fprintf('Error en sim(): %s\n', ME.message);
        cost = 1e10; 
    end
end