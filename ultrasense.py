#Laura Calderón
# Giuliana Volpi
import sys
import heapq


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
        
def obtener_numeros_unicos(parejas):

    numeros_unicos = set()
    for a, b in parejas:
        numeros_unicos.add(a)
        numeros_unicos.add(-a)
        numeros_unicos.add(b)
        numeros_unicos.add(-b)

    lista_numeros_unicos = list(numeros_unicos)
    lista_numeros_unicos.sort()

    return lista_numeros_unicos


def calcular_matriz_energia(lista_numeros_unicos, w1, w2):
    tam = len(lista_numeros_unicos)
    costos = [[float('inf') for _ in range(tam)] for _ in range(tam)] 

    for i in range(tam):
        for j in range(tam):
            m1 = lista_numeros_unicos[i]
            m2 = lista_numeros_unicos[j]

            if m1 == m2 or m1 == -m2:
                continue

            c1 = 1 if m1 >= 0 else -1
            c2 = 1 if m2 >= 0 else -1

            if c1 == c2:
                ltp = 1 + abs(abs(m1) - abs(m2)) % w1
            else:
                ltp = w2 - abs(abs(m1) - abs(m2)) % w2

            costos[i][j] = ltp

    return costos

def encontrar_camino_euleriano(grafo):
    nodos_impar = [nodo for nodo, vecinos in grafo.items() if len(vecinos) % 2 != 0]
    if len(nodos_impar) not in [0, 2]:
        return None

    vert_inicio = nodos_impar[0] if len(nodos_impar) == 2 else next(iter(grafo)) 
    pila = [vert_inicio]
    camino = []

    while pila:
        vert = pila[-1]
        if not grafo[vert]:
            camino.append(pila.pop())
        else:
            vert_siguiente = grafo[vert].pop()
            grafo[vert_siguiente].remove(vert)
            pila.append(vert_siguiente)
            
    for vecinos in grafo.values():
        if vecinos:
            return None

    camino_parejas = [(camino[i], camino[i+1]) for i in range(len(camino) - 1)]
    
    return camino_parejas

def dijkstra(matriz_energia, lista_numeros_unicos, camino_parejas):
    caminos = []  
    total_ltp = 0  

    def get_index(num):
        return lista_numeros_unicos.index(num)

    for i in range(len(camino_parejas) - 1):
        inicio = get_index(camino_parejas[i][1])
        final = get_index(-camino_parejas[i+1][0])
        distancias = [float('inf')] * len(lista_numeros_unicos)
        distancias[inicio] = 0  
        heap = [(0, inicio, [lista_numeros_unicos[inicio]])]
        
        while heap:
            (costo, nodo_actual, camino) = heapq.heappop(heap)
            if nodo_actual == final:
                caminos.append(camino)  
                total_ltp += costo
                break
            for vecino in range(len(lista_numeros_unicos)):
                if vecino != nodo_actual:
                    nuevo_costo = costo + matriz_energia[nodo_actual][vecino]
                    if nuevo_costo < distancias[vecino]:
                        distancias[vecino] = nuevo_costo
                        heapq.heappush(heap, (nuevo_costo, vecino, camino + [lista_numeros_unicos[vecino]]))
                    
    return caminos, total_ltp



def main():
    num_casos = int(sys.stdin.readline().strip())
    print(f"Cantidad de casos: {num_casos}")

    for caso in range(1, num_casos + 1):
        n, w1, w2 = map(int, sys.stdin.readline().strip().split())
        parejas = [tuple(map(int, sys.stdin.readline().strip().split())) for _ in range(n)]
        grafo = crear_grafo(parejas)
        camino = encontrar_camino_euleriano(grafo)

        if camino is None:
            print(f"Caso {caso}: No se encontró camino euleriano")
            continue

        unicos = obtener_numeros_unicos(parejas)
        costos = calcular_matriz_energia(unicos, w1, w2)
        rutas, total_ltp = dijkstra(costos, unicos, camino)

        print(f"Caso {caso}: n={n}, w1={w1}, w2={w2}")
        resultado = []
        for i, ruta in enumerate(rutas):
            if i < len(camino) - 1:
                resultado.append(f"({camino[i][0]},{camino[i][1]}),{','.join(map(str, ruta))}")
        print(",".join(resultado) + f" {total_ltp}")

if __name__ == "__main__":
    main()
