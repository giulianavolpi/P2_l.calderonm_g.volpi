#Laura Calderón
# Giuliana Volpi
import sys
import heapq

# def dijkstra(graph, start):
#     distances = {node: float('inf') for node in graph}
#     distances[start] = 0
#     pq = [(0, start)]  # Tupla (distancia_actual, nodo)
#     while pq:
#         current_distance, current_node = heapq.heappop(pq)
#         if current_distance > distances[current_node]:
#             continue
#         for neighbor, weight in graph[current_node].items():
#             distance = current_distance + weight
#             if distance < distances[neighbor]:
#                 distances[neighbor] = distance
#                 heapq.heappush(pq, (distance, neighbor))

#     return distances


# def dijkstra_algorithm(graph, start_node):
#     unvisited_nodes = list(graph.get_nodes())  
#     shortest_path = {}
#     previous_nodes = {} 
#     max_value = sys.maxsize
#     for node in unvisited_nodes:
#         shortest_path[node] = max_value 
#     shortest_path[start_node] = 0
#     while unvisited_nodes:
#         current_min_node = None
#         for node in unvisited_nodes: # Iterate over the nodes
#             if current_min_node == None:
#                 current_min_node = node
#             elif shortest_path[node] < shortest_path[current_min_node]:
#                 current_min_node = node
#         neighbors = graph.get_outgoing_edges(current_min_node)
#         for neighbor in neighbors:
#             tentative_value = shortest_path[current_min_node] + graph.value(current_min_node, neighbor)
#             if tentative_value < shortest_path[neighbor]:
#                 shortest_path[neighbor] = tentative_value
#                 previous_nodes[neighbor] = current_min_node
#         unvisited_nodes.remove(current_min_node)
    
#     return previous_nodes, shortest_path


def crear_grafo(parejas):
    grafo = {}
    for a, b in parejas:
        if a not in grafo:
            grafo[a] = set()
        if b not in grafo:
            grafo[b] = set()
        grafo[a].add(b)
        grafo[b].add(a)
    return grafo

def mostrar_grafo(grafo):
    for nodo, adyacentes in grafo.items():
        print(f"{nodo}: {list(adyacentes)}")

def encontrar_camino_euleriano(grafo):

    nodos_impar = [nodo for nodo, vecinos in grafo.items() if len(vecinos) % 2 != 0]
    
    if len(nodos_impar) not in [0, 2]:
        return None

    start_vertex = nodos_impar[0] if len(nodos_impar) == 2 else next(iter(grafo)) 
    stack = [start_vertex]
    path = []

    while stack:
        vertex = stack[-1]
   
        if not grafo[vertex]:
            path.append(stack.pop())
        else:
            next_vertex = grafo[vertex].pop()
            grafo[next_vertex].remove(vertex)
            stack.append(next_vertex)

    camino_parejas = [(path[i], path[i+1]) for i in range(len(path) - 1)]
    return camino_parejas





def main():
    num_casos = int(sys.stdin.readline().strip())
    print(f"Cantidad de casos: {num_casos}")

    for caso in range(1, num_casos + 1):
        n, w1, w2 = map(int, sys.stdin.readline().strip().split())
        print(f"\nCaso {caso}:")
        print(f"n={n}, w1={w1}, w2={w2}")

        parejas = [tuple(map(int, sys.stdin.readline().strip().split())) for _ in range(n)]
        print(f"Parejas: {parejas}")

        grafo = crear_grafo(parejas)
        
        camino = encontrar_camino_euleriano(grafo)
        print(f"Camino Euleriano: {camino}")
        

        print(f"Grafo del Caso {caso}:")
        mostrar_grafo(grafo)

if __name__ == "__main__":
    main()

