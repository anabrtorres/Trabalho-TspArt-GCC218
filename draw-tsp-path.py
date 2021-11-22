"""Modified code from https://developers.google.com/optimization/routing/tsp#or-tools """
# Copyright Matthew Mack (c) 2020 under CC-BY 4.0: https://creativecommons.org/licenses/by/4.0/

from __future__ import print_function
import math 
from math import *
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from PIL import Image, ImageDraw
import random
import time

# Change these file names to the relevant files.
ORIGINAL_IMAGE = "images/ufla-logo.png"
IMAGE_TSP = "images/ufla-logo-1024-stipple.tsp"

def create_data_model():
    """Stores the data for the problem."""
    # Extracts coordinates from IMAGE_TSP and puts them into an array
    list_of_nodes = []
    with open(IMAGE_TSP) as f:
        for _ in range(6):
            next(f)
        for line in f:
            i,x,y = line.split()
            list_of_nodes.append((int(float(x)),int(float(y))))
    data = {}
    # Locations in block units
    data['locations'] = list_of_nodes # yapf: disable
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data

def compute_euclidean_distance_matrix(locations):
    """Creates callback to return distance between points."""
    distances = {}
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                # Euclidean distance
                distances[from_counter][to_counter] = (int(
                    math.hypot((from_node[0] - to_node[0]),
                               (from_node[1] - to_node[1]))))
    return distances

def print_solution(manager, routing, solution):
    """Prints solution on console."""
    print('Objective: {}'.format(solution.ObjectiveValue()))
    index = routing.Start(0)
    plan_output = 'Route:\n'
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    plan_output += 'Objective: {}m\n'.format(route_distance)

def get_routes(solution, routing, manager):
    """Get vehicle routes from a solution and store them in an array."""
    # Get vehicle routes and store them in a two dimensional array whose
    # i,j entry is the jth location visited by vehicle i along its route.
    routes = []    
    for route_nbr in range(routing.vehicles()):      
      index = routing.Start(route_nbr)           
      route = [manager.IndexToNode(index)]     

      while not routing.IsEnd(index):
        index = solution.Value(routing.NextVar(index))        
        route.append(manager.IndexToNode(index))       

      routes.append(route)      
    return routes[0]

def draw_routes(nodes, path):
    """Takes a set of nodes and a path, and outputs an image of the drawn TSP path"""
    tsp_path = []
    
    for location in path:        
        tsp_path.append(nodes[int(location)])

    original_image = Image.open(ORIGINAL_IMAGE)
    width, height = original_image.size

    tsp_image = Image.new("RGBA",(width,height),color='white')
    tsp_image_draw = ImageDraw.Draw(tsp_image)
    #tsp_image_draw.point(tsp_path,fill='black')
    tsp_image_draw.line(tsp_path,fill='black',width=1)
    tsp_image = tsp_image.transpose(Image.FLIP_TOP_BOTTOM)
    FINAL_IMAGE = IMAGE_TSP.replace("-stipple.tsp","-tsp.png")
    tsp_image.save(FINAL_IMAGE)
    print("TSP solution has been drawn and can be viewed at", FINAL_IMAGE)

def vizinhoMaisProximo(matrizDist, v):
    visitado = [v]
    rota = [v]
       
    while len(visitado) != len(matrizDist[v]): #enquanto todos os vertices não forem visitados
        minDist = float('Inf')           
        for x in matrizDist[v]: #percorre os adjacentes do vertice         
            if x not in visitado:               
               d = matrizDist[v][x] #retorna as distancias dos adjacentes                             
               if d < minDist : #obtem a menor distancia, o vizinho mais proximo.
                  vertice_visitado = x                 
                  minDist = d                                  
                          
        visitado.append(vertice_visitado)
        rota.append(vertice_visitado)
        v = vertice_visitado
    rota.append(0)                  
    return rota 
                 
def custoMudanca(matrizDist, n1, n2, n3, n4):
    return matrizDist[n1][n3] + matrizDist[n2][n4] - matrizDist[n1][n2] - matrizDist[n3][n4]

def two_opt(route, matrizDist):
    melhorRota = route
    otimizado = True
    while otimizado:
        otimizado = False
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route)):
                if j - i == 1: 
                    continue
                if custoMudanca(matrizDist, melhorRota[i - 1],melhorRota[i], melhorRota[j - 1], melhorRota[j]) < 0:
                    melhorRota[i:j] = melhorRota[j - 1:i - 1:-1]
                    otimizado = True
        route = melhorRota
    return melhorRota

#Gera uma rota aleatório
def randomRoute(n):
    routes = list(range(n))
    random.shuffle(routes)
    return routes

def main():
    """Entry point of the program."""
    print('\n Qual algoritmo deseja executar? \n '
                    '1 - TSP Art (Original) \n '
                    '2 - Vizinho Mais Proximo \n '
                    '3 - Vizinho Mais Proximo + 2OPT \n '
                    '4 - Random + 2OPT \n ')
    opc = int(input("Selecione: "))

    # Instantiate the data problem.
    print("Step 1/5: Initialising variables")
    data = create_data_model()
    
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['locations']), data['num_vehicles'], data['depot'])    
   
    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)
    
    print("Step 2/5: Computing distance matrix")
    distance_matrix = compute_euclidean_distance_matrix(data['locations'])
    
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    def custo(rota):                
        custoTotal = 0
        for i in range(len(rota) - 1):
           custoTotal += distance_matrix[rota[i]][rota[i+1]]
        return custoTotal

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)  
    
    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)    

    if opc == 1:
        # Setting first solution heuristic.
        print("Step 3/5: Setting an initial solution")
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

        # Solve the problem.
        print("Step 4/5: Solving")        
        solution = routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        if solution:        
            print("Step 5/5: TSP-Art - Drawing the solution")
            inicio = time.time()          
            tsp_routes = get_routes(solution, routing, manager)
            fim = time.time()            
            draw_routes(data['locations'], tsp_routes)
            print("Tempo de Execução: ",fim - inicio)
            print("Distância Percorrida: ", custo(tsp_routes) )
        else:
            print("A solution couldn't be found :(")

    if opc == 2:
        print("Step 3/5: Setting an initial solution")
        print("Step 4/5: Solving")
        inicio = time.time()        
        nn_routes = vizinhoMaisProximo(distance_matrix, 0)
        fim = time.time()        
        print("Step 5/5: Nearest Neighbour - Drawing the solution")
        draw_routes(data['locations'], nn_routes) 
        print("Tempo de Execução: ",fim - inicio)
        print("Distância Percorrida: ", custo(nn_routes) )
       
    if opc == 3:
        print("Step 3/5: Setting an initial solution")
        print("Step 4/5: Solving") 
        inicio = time.time()      
        nn_routes = vizinhoMaisProximo(distance_matrix, 0)
        #inicio = time.time() 
        opt_routes = two_opt(nn_routes, distance_matrix) 
        fim = time.time()               
        print("Step 5/5: Nearest Neighbour and 2OPT - Drawing the solution")
        draw_routes(data['locations'], opt_routes)
        print("Tempo de Execução: ",fim - inicio)
        print("Distância Percorrida: ", custo(opt_routes) )
        
    if opc == 4:
        print("Step 3/5: Setting an initial solution")
        print("Step 4/5: Solving") 
        #inicio = time.time() 
        ran_route =  randomRoute(len(distance_matrix[0]))
        inicio = time.time()
        r_routes = two_opt(ran_route, distance_matrix)
        fim = time.time()               
        print("Step 5/5: Random and 2OPT - Drawing the solution")        
        draw_routes(data['locations'], r_routes)
        print("Tempo de Execução: ",fim - inicio)
        print("Distância Percorrida: ", custo(r_routes) )


if __name__ == '__main__':
    main()