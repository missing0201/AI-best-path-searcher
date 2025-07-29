function main
    % Load map
    map = im2bw(imread('random_map.bmp'));
    [nRows, nCols] = size(map);
    start = [1, 1];
    finish = [500, 500];
    
    % variables
    nPoints = 10; 
    populationSize = 100;
    maxGenerations = 500;

    % User input text for GA configuration
    selectionMethod = input('Select method (0=RWS, 1=Tournament, 2=RBS): ');
    crossoverMethod = input('Select crossover (0=1-point, 1=Uniform): ');
    mutationMethod = input('Select mutation (0=Gaussian, 1=Random Reset): ');

    % Run GA
    [bestPath, execTime, bestDistance] = runGA(map, start, finish, nPoints,populationSize, maxGenerations,selectionMethod, crossoverMethod, ...
                                               mutationMethod);

    % Display Results
    fprintf('Run Time: %.2f s\n', execTime);
    fprintf('Best Path Distance: %.2f units\n', bestDistance);

    % Plot best path map
    plotPath(map, bestPath);
end

% Genetic Algorithm runner
function [bestPath, execTime, bestDistance] = runGA(map, start, finish, nPoints, ...
                                                   populationSize, maxGenerations, ...
                                                   selectionMethod, crossoverMethod, mutationMethod)
    tic; % Start timer

    % Initialize population
    population = loadPopulation(populationSize, nPoints, size(map));

    % Main GA loop
    for generation=1:maxGenerations
        % Evaluate fitness
        fitness = evaluateFitness(population, map, start, finish, nPoints);

        % Selection process
        parents = selection(population, fitness, selectionMethod);

        % Crossover methods
        offspring = crossover(parents, crossoverMethod);
        
        % Mutation
        offspring = mutation(offspring, mutationMethod, size(map));

        % Replace population with offsprings
        population = offspring;
    end

    % Get best path
    fitness = evaluateFitness(population, map, start, finish, nPoints);
    [bestFitness, bestIndex] = min(fitness);
    bestPath = population(bestIndex, :);

    % Output
    execTime = toc; % Stop timer
    bestDistance = bestFitness;
end

% population initializer
function population = loadPopulation(populationSize, nPoints, mapSize)
    % generate points for the path
    interPoints = randi([1, mapSize(1)], populationSize, (nPoints - 2)*2);
    
    % Add Start Point and End Point
    startPoint = repmat([1, 1], populationSize, 1);
    endPoint = repmat([500,500], populationSize, 1);
    
    % Construct the population
    population = [startPoint, interPoints, endPoint];
    return
end

%Eucludian Distance method
function distance=eucDistance(PointFX,PointSX,PointFY,PointSY)
distance=sqrt((PointFX-PointSX).^2+(PointFY-PointSY).^2);
end

% Fitness Function
function fitness = evaluateFitness(population, map, start, finish, nPoints)
    numIndividuals = size(population, 1);
    fitness = zeros(numIndividuals, 1);

    for i = 1:numIndividuals

        currentPath=population(i,:);

        distance1=eucDistance(currentPath(1),currentPath(3),currentPath(2),currentPath(4));

        distance2=eucDistance(currentPath(3),currentPath(5),currentPath(4),currentPath(6));

        distance3=eucDistance(currentPath(5),currentPath(7),currentPath(6),currentPath(8));

        distance4=eucDistance(currentPath(7),currentPath(9),currentPath(8),currentPath(10));

        distance5=eucDistance(currentPath(9),currentPath(11),currentPath(10),currentPath(12));

        distance6=eucDistance(currentPath(11),currentPath(13),currentPath(12),currentPath(14));

        distance7=eucDistance(currentPath(13),currentPath(15),currentPath(14),currentPath(16));

        distance8=eucDistance(currentPath(15),currentPath(17),currentPath(16),currentPath(18));

        distance9=eucDistance(currentPath(17),currentPath(19),currentPath(18),currentPath(20));
        
        pathLength=distance1+distance2+distance3+distance4+distance5+distance6+distance7+distance8+distance9;

        
        penalty = 0;
        
        % Check for intersection with obstacles
        start=[currentPath(1),currentPath(2)];
        finish=[currentPath(3),currentPath(4)];
        if intersects_obstacle(start,finish,map)
            penalty = penalty + distance1;
        end

        start=[currentPath(3),currentPath(4)];
        finish=[currentPath(5),currentPath(6)];
        if intersects_obstacle(start,finish,map)
            penalty = penalty + distance2;
        end

        start=[currentPath(5),currentPath(6)];
        finish=[currentPath(7),currentPath(8)];
        if intersects_obstacle(start,finish,map)
            penalty = penalty + distance3;
        end

        start=[currentPath(7),currentPath(8)];
        finish=[currentPath(9),currentPath(10)];
        if intersects_obstacle(start,finish,map)
            penalty = penalty + distance4;
        end

        start=[currentPath(9),currentPath(10)];
        finish=[currentPath(11),currentPath(12)];
        if intersects_obstacle(start,finish,map)
            penalty = penalty + distance5;
        end

        start=[currentPath(11),currentPath(12)];
        finish=[currentPath(13),currentPath(14)];
        if intersects_obstacle(start,finish,map)
            penalty = penalty + distance6;
        end

        start=[currentPath(13),currentPath(14)];
        finish=[currentPath(15),currentPath(16)];
        if intersects_obstacle(start,finish,map)
            penalty = penalty + distance7;
        end

        start=[currentPath(15),currentPath(16)];
        finish=[currentPath(17),currentPath(18)];
        if intersects_obstacle(start,finish,map)
            penalty = penalty + distance8;
        end

        start=[currentPath(17),currentPath(18)];
        finish=[currentPath(19),currentPath(20)];
        if intersects_obstacle(start,finish,map)
            penalty = penalty + distance9;
        end
        
        % fitness calculation
        fitness(i) =  pathLength+ penalty;
    end
end

%Obstacle intersection checker
function is_intersecting= intersects_obstacle(start,finish,map)
is_intersecting=false;
x1=round(start(1));
y1=round(start(2));
x2=round(finish(1));
y2=round(finish(2));
dx=abs(x2-x1);
dy=abs(y2-y1);
sx=sign(x2-x1);
sy=sign(y2-y1);
err=dx-dy;

while true
    if map(x1,y1)==0
       is_intersecting=true;
       return;
    end

    if x1==x2 && y1==y2
        break;
    end

    e2=2*err;

    if e2>-dy
        err=err-dy;
        x1=x1+sx;
    end

    if e2<dx
        err=err+dx;
        y1=y1+sy;
    end
end

is_intersecting=false;
end

% Selection Function
function parents = selection(population, fitness, method)
    pSize = size(population, 1);
    parents = zeros(size(population));

    switch method
        case 0 % Roulette Wheel Selection
            prob = 1 ./ fitness;
            prob = prob / sum(prob);
            for i = 1:pSize
                index = find(rand <= cumsum(prob), 1);
                parents(i, :) = population(index, :);
            end

        case 1 % Tournament Selection
            tournaments = 3;
            for i = 1:pSize
                indices = randperm(pSize, tournaments);
                [~, bestIdx] = min(fitness(indices));
                parents(i, :) = population(indices(bestIdx), :);
            end

        case 2 % Rank Based Selection
            [~, sortedIndices] = sort(fitness); 
            ranks = pSize - (1:pSize) + 1; 
            prob = 1-(ranks / sum(ranks)); 
    
            for i = 1:pSize
                index = find(rand <= cumsum(prob(sortedIndices)), 1);
                parents(i, :) = population(sortedIndices(index), :);
            end
    end
end

% Function for crossover
function offspring = crossover(parents, method)
    numIndividuals = size(parents, 1);
    offspring = zeros(size(parents));

    for i = 1:2:numIndividuals-1
        parent1 = parents(i, :);
        parent2 = parents(i+1, :);

        switch method
            case 0 % 1-point crossover
                point = randi([3, length(parent1) - 3]);
                offspring(i, :) = [parent1(1:point), parent2(point+1:end)];
                offspring(i+1, :) = [parent2(1:point), parent1(point+1:end)];
            case 1 %Uniform crossover
                for x=1:length(offspring(1,:))
                    CoinFlip=randi(2);
                    if CoinFlip==1
                        offspring(i, x) = parent1(x);
                        offspring(i+1, x) = parent2(x);
                    end
                    if CoinFlip==2
                        offspring(i, x) = parent2(x);
                        offspring(i+1, x) = parent1(x);
                    end
                end
        end
    end
end

% Function for mutation
function mutants = mutation(offspring, method, mapSize)
    numIndividuals = size(offspring, 1);
    mutants = offspring;
    possibility=randi(10);
    for i = 1:numIndividuals
        switch method
            case 0 % Gaussian Mutation
                if possibility<5
                random=round(randn(1,length(offspring(i, :))));
                random(1:2)=0;
                random(end-1:end)=0;
                mutants(i, :) = mutants(i, :) + random;
                end
            case 1 % Random Reset
                if possibility<5
                index = randi([3,length(offspring(i, :))-3]);
                mutants(i, index) = randi([1, mapSize(1)]);
                end
        end
        mutants(i, :) = max(1, min(mapSize(1), mutants(i, :))); % Ensure within bounds
    end
end

% best path plotting function
function plotPath(map, bestPath)
    clf;
    imshow(map); hold on;

    path = reshape(bestPath,2,[]);

    line(path(2,:),path(1,:), 'Color', 'r', 'LineWidth', 2);
end