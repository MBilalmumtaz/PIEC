import numpy as np

def dominates(a, b):
    """Return True if a dominates b (all objectives better or equal, at least one strictly better)"""
    return all(x <= y for x, y in zip(a, b)) and any(x < y for x, y in zip(a, b))


def fast_non_dominated_sort(objectives):
    """Fast non-dominated sort returning list of fronts, each front is a list of indices"""
    S = [[] for _ in range(len(objectives))]
    n = [0]*len(objectives)
    rank = [0]*len(objectives)
    fronts = [[]]

    for p in range(len(objectives)):
        for q in range(len(objectives)):
            if p == q:
                continue
            if dominates(objectives[p], objectives[q]):
                S[p].append(q)
            elif dominates(objectives[q], objectives[p]):
                n[p] += 1
        if n[p] == 0:
            rank[p] = 0
            fronts[0].append(p)

    i = 0
    while fronts[i]:
        next_front = []
        for p in fronts[i]:
            for q in S[p]:
                n[q] -= 1
                if n[q] == 0:
                    rank[q] = i + 1
                    next_front.append(q)
        i += 1
        fronts.append(next_front)

    return fronts[:-1]


def crowding_distance(front, objectives):
    """Calculate crowding distance for individuals in a front"""
    if not front:
        return {}
    
    # Convert front to list if needed
    front_list = list(front)
    
    # Initialize distances
    distance = {i: 0.0 for i in front_list}
    
    # For each objective dimension
    num_objectives = len(objectives[0])
    
    for m in range(num_objectives):
        # Sort by current objective
        front_list.sort(key=lambda i: objectives[i][m])
        
        # Set boundary points to infinity
        distance[front_list[0]] = float('inf')
        distance[front_list[-1]] = float('inf')
        
        # Calculate range for normalization
        min_val = objectives[front_list[0]][m]
        max_val = objectives[front_list[-1]][m]
        span = max_val - min_val
        
        if span < 1e-9:
            continue
            
        # Add distance for interior points
        for i in range(1, len(front_list)-1):
            prev = front_list[i-1]
            curr = front_list[i]
            next_i = front_list[i+1]
            
            distance[curr] += (objectives[next_i][m] - objectives[prev][m]) / span
    
    return distance


def nsga2_selection(population, fitness_values, pop_size):
    """Main NSGA-II selection operator with error handling"""
    # Handle empty cases
    if not fitness_values or not population:
        return list(range(min(len(population), pop_size)))
    
    # Non-dominated sorting
    try:
        fronts = fast_non_dominated_sort(fitness_values)
    except Exception as e:
        # If sorting fails, return random selection
        if len(population) <= pop_size:
            return list(range(len(population)))
        else:
            return np.random.choice(len(population), pop_size, replace=False).tolist()
    
    # Handle empty fronts case
    if not fronts:
        if len(population) <= pop_size:
            return list(range(len(population)))
        else:
            return np.random.choice(len(population), pop_size, replace=False).tolist()
    
    # Selection
    selected = []
    front_idx = 0
    
    while len(selected) + len(fronts[front_idx]) <= pop_size and front_idx < len(fronts):
        # Add entire front
        selected.extend(fronts[front_idx])
        front_idx += 1
        if front_idx >= len(fronts):
            break
    
    # Fill remaining spots with crowding distance
    if len(selected) < pop_size and front_idx < len(fronts):
        last_front = fronts[front_idx]
        distances = crowding_distance(last_front, fitness_values)
        
        # Sort by crowding distance (descending)
        sorted_indices = sorted(last_front, key=lambda i: distances.get(i, 0), reverse=True)
        
        # Add best individuals from last front
        needed = pop_size - len(selected)
        selected.extend(sorted_indices[:needed])
    elif len(selected) < pop_size:
        # If we still need more, add random individuals
        needed = pop_size - len(selected)
        all_indices = list(range(len(population)))
        available_indices = [i for i in all_indices if i not in selected]
        if len(available_indices) >= needed:
            selected.extend(np.random.choice(available_indices, needed, replace=False))
        else:
            selected.extend(available_indices)
    
    return selected[:pop_size]
