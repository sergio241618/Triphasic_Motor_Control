function [Kp_best, Ki_best, Kd_best, gbest_cost] = PSO_Fuzzy_Tuning()
    clc; clear; close all;
    
    % =====================================================================
    % 1. CONFIGURACIÓN
    % =====================================================================
    % ¡IMPORTANTE! Pon aquí el nombre EXACTO de tu archivo Simulink DIFUSO
    model_name = 'PSO_PID_dif_Frecuencia'; 
    
    num_particles = 3;
    max_iter = 3;
    
    % --- RANGOS DE BÚSQUEDA ---
    % Mapeo: x(1)=Proporcional, x(2)=Integral, x(3)=Derivativo
    % En Fuzzy a veces se requieren ganancias más grandes que en PID normal.
    VarMin = [0.0   0.0   0.0];   
    VarMax = [1000.0  1000.0  1000.0];  
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
    
    % --- BUCLE INICIAL ---
    fprintf('Inicializando partículas para Fuzzy PID...\n');
    for i = 1:num_particles
        particle(i).pos = VarMin + rand(1,nVar).*(VarMax - VarMin);
        particle(i).vel = zeros(1,nVar);
        
        % Evaluar costo
        particle(i).cost = cost_function_fuzzy(particle(i).pos, model_name);
        
        particle(i).best.pos = particle(i).pos;
        particle(i).best.cost = particle(i).cost;
        
        if particle(i).best.cost < GlobalBest.cost
            GlobalBest = particle(i).best;
        end
        % Imprimir estado para verificar que no esté fallando instantáneamente
        fprintf('  Partícula %d: Costo=%.4f (Kp=%.2f Ki=%.2f Kd=%.2f)\n', ...
            i, particle(i).cost, particle(i).pos(1), particle(i).pos(2), particle(i).pos(3));
    end
    
    % --- BUCLE PRINCIPAL ---
    fprintf('\nIniciando optimización...\n');
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
            particle(i).cost = cost_function_fuzzy(particle(i).pos, model_name);
            
            % Actualizar PBest y GBest
            if particle(i).cost < particle(i).best.cost
                particle(i).best.pos = particle(i).pos;
                particle(i).best.cost = particle(i).cost;
                if particle(i).best.cost < GlobalBest.cost
                    GlobalBest = particle(i).best;
                end
            end
        end
        fprintf('Iteración %d | Mejor Costo: %.4f | Kp: %.3f | Ki: %.3f | Kd: %.3f\n', ...
            it, GlobalBest.cost, GlobalBest.pos(1), GlobalBest.pos(2), GlobalBest.pos(3));
    end
    
    % --- RESULTADOS ---
    Kp_best = GlobalBest.pos(1);
    Ki_best = GlobalBest.pos(2);
    Kd_best = GlobalBest.pos(3);
    gbest_cost = GlobalBest.cost;
    
    % Asignar variables al workspace para prueba manual
    assignin('base', 'K_fp', Kp_best);
    assignin('base', 'K_fi', Ki_best);
    assignin('base', 'K_fd', Kd_best);
    
    disp('Optimización Completa.');
    disp(['Mejor K_fp (Prop): ', num2str(Kp_best)]);
    disp(['Mejor K_fi (Int):  ', num2str(Ki_best)]);
    disp(['Mejor K_fd (Der):  ', num2str(Kd_best)]);
end

% =====================================================================
% FUNCIÓN DE COSTO ADAPTADA A VARIABLES DIFUSAS
% =====================================================================
function cost = cost_function_fuzzy(x, model_name)
    % 1. Configuración de la simulación
    simIn = Simulink.SimulationInput(model_name);
    
    % AQUÍ ESTÁ LA CLAVE: Asignamos a las variables difusas
    simIn = simIn.setVariable('K_fp', x(1)); % Proporcional
    simIn = simIn.setVariable('K_fi', x(2)); % Integral
    simIn = simIn.setVariable('K_fd', x(3)); % Derivativo
    
    try
        % 2. Ejecutar Simulación
        out = sim(simIn);
        
        % 3. BÚSQUEDA INTELIGENTE DE LA VARIABLE (Igual que el PID que funcionó)
        e = []; 
        
        % Opción A: Array/Timeseries directo
        if isprop(out, 'error_sim')
            raw = out.error_sim;
            if isa(raw, 'timeseries'), e = raw.Data; else, e = raw; end
            
        % Opción B: Dataset (logsout)
        elseif isprop(out, 'logsout') && ~isempty(out.logsout.find('error_sim'))
            e = out.logsout.get('error_sim').Values.Data;
            
        % Opción C: Método get()
        elseif isa(out, 'Simulink.SimulationOutput') && ~isempty(out.find('error_sim'))
            raw = out.get('error_sim');
            if isa(raw, 'timeseries'), e = raw.Data; else, e = raw; end
        end
        
        % 4. Verificación final
        if isempty(e)
            % Si falla, imprimimos mensaje para depurar en consola
            disp('--- ERROR: No se encontraron datos de error_sim ---'); 
            cost = 1e10; 
            return;
        end

        % 5. Cálculo del Costo (MSE)
        if any(isnan(e)) || any(isinf(e))
            cost = 1e10;
        else
            cost = mean(e.^2);
        end
        
    catch ME
        % Si falla simulink, mostramos por qué
        fprintf('Error en simulación: %s\n', ME.message);
        cost = 1e10; 
    end
end