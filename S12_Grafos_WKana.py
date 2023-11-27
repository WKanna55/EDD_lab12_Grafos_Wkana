import heapq

"""Ejercicio 01: Implementacion de un grafo no dirigido (inicio)"""
class Nodo:
    def __init__(self, etiqueta) -> None:
        self.etiqueta = etiqueta

    def __str__(self):
        return self.etiqueta

class Grafo:
    def __init__(self) -> None:
        self.nodos = {}
        self.lista_adyacencia = {}

    def agregar_nodo(self, etiqueta):
        if etiqueta not in self.nodos:
            self.nodos[etiqueta] = Nodo(etiqueta)
            self.lista_adyacencia[etiqueta] = [] # iniciar una lista vacia

    def eliminar_nodo(self, etiqueta):
        nodo = self.nodos[etiqueta]
        for clave in self.lista_adyacencia:
            if nodo in self.lista_adyacencia[clave]:
                self.lista_adyacencia[clave].remove(nodo)
        del self.lista_adyacencia[etiqueta]
        del self.nodos[etiqueta]

    def agregar_arista(self, fuente, destino):

        self.lista_adyacencia[fuente].append(self.nodos[destino])

    def eliminar_arista(self, fuente, destino):
        self.lista_adyacencia[fuente].remove(self.nodos[destino])

    def imprimir_grafo(self):
        for nodo in self.lista_adyacencia:
            print(nodo, end="==>")
            for adyacente in self.lista_adyacencia[nodo]:
                print(adyacente.etiqueta, end=', ')
            print()


    """Ejercicio 01: Implementacion de un grafo no dirigido (fin)"""

    """Ejercicio 02: Funcion existe camino de nodo "a" a nodo "n" (inicio)"""
    def esta_conectado(self, fuente, destino):
        verificador = False
        if fuente in self.lista_adyacencia:
            for i in self.lista_adyacencia[fuente]:
                if destino == i.etiqueta:
                    verificador = True
                    break
            return verificador
        else:
            return verificador
    """Ejercicio 02: Funcion existe camino de nodo "a" a nodo "n" (fin)"""

    """Ejercicio 03: DFS y BFS (inicio)"""
    def bfs(self, nodo):
        cola = [nodo]
        visitado = []
        while len(cola) > 0:
            actual = cola.pop(0)
            if actual in visitado:
                continue # si ya esta visitado omitir todoo lo
                         # demas debajo de esta linea e iniciar una nueva iteracion
            visitado.append(actual)
            for vecino in self.lista_adyacencia[actual]:
                if vecino.etiqueta not in visitado:
                    cola.append(vecino.etiqueta)
        return visitado

    def dfs(self, nodo):
        pila = [nodo]
        visitado = []
        while len(pila) > 0:
            actual = pila.pop(-1)
            if actual in visitado:
                continue # si ya esta visitado omitir todoo lo
                         # demas debajo de esta linea e iniciar una nueva iteracion
            visitado.append(actual)
            for vecino in self.lista_adyacencia[actual]:
                if vecino.etiqueta not in visitado:
                    pila.append(vecino.etiqueta)
        return visitado
    """Ejercicio 03: DFS y BFS (fin)"""

    """Ejercicio 04: Grado de vetice(nodo)(conexiones) (inicio)"""
    def grado_nodo(self, nodo):
        contador = 0
        if nodo in self.lista_adyacencia:
            for i in self.lista_adyacencia[nodo]:
                contador += 1
            return contador
        else:
            raise Exception("Error: nodo invalido, no creado")

    """Ejercicio 04: Grado de vetice(nodo)(conexiones) (fin)"""

    """Ejercicio 06: Detectar ciclos en un grafo dirigido (inicio)"""
    def ciclos_Tarjan(self, nodo):
        contar_ciclos = 0
        pila = [nodo]
        visitado = []
        while len(pila) > 0:
            actual = pila.pop(-1)
            if actual in visitado:
                continue  # si ya esta visitado omitir todoo lo
                # demas debajo de esta linea e iniciar una nueva iteracion
            visitado.append(actual)
            for vecino in self.lista_adyacencia[actual]:
                if vecino.etiqueta not in visitado:
                    pila.append(vecino.etiqueta)
                else:
                    contar_ciclos += 1
        return contar_ciclos

    """Ejercicio 06: Detectar ciclos en un grafo dirigido (fin)"""

    """Ejercicio 08: Encontrar componentes conectados en un grafo no dirigido (inicio)"""
    def componentes(self, nodo):
        componentes = []
        valores = self.dfs(nodo)
        componentes.append(valores)
        for i in self.lista_adyacencia:
            for j in componentes:
                if i not in j:
                    componentes.append(self.dfs(i))
                    break
        return componentes

    """Ejercicio 08: Encontrar componentes conectados en un grafo no dirigido (fin)"""

"""Ejercicio 05: Camino corto entre dos vertices (inicio) / lab13"""
class Node:
    def __init__(self, label) -> None:
        self.label = label
    def __str__(self) -> str:
        return f"{self.label}"

class Edge:
    def __init__(self, source, destination, weight) -> None:
        self.source = Node(source)
        self.destination = Node(destination)
        self.weight = weight

class WeightedGraph:
    def __init__(self) -> None:
        self.nodes = {}
        self.edges = {}

    def add_node(self, label):
        self.nodes[label] = Node(label)
        self.edges[label] = []

    def add_edge(self, source, destination, weight):
        self.edges[source].append(Edge(source, destination, weight))
        self.edges[destination].append(Edge(destination, source, weight))

    def dijkstra(self, source, destination):
        distances = {}

        for node in self.nodes:
            distances[node] = float('inf')
        distances[source] = 0

        previous_nodes = {}
        visited = []
        queue = [(0, source)]

        while len(queue) > 0:
            current = heapq.heappop(queue)[1]
            visited.append(current)

            for edge in self.edges[current]:
                if edge.destination.label in visited:
                    continue
                new_distance = distances[current] + edge.weight
                if new_distance < distances[edge.destination.label]:
                    distances[edge.destination.label] = new_distance
                    previous_nodes[edge.destination.label] = current
                    queue.append((new_distance, edge.destination.label))

        return self.build_path(destination, previous_nodes)

    def build_path(self, destination, previous_nodes):
        stack = [self.nodes[destination].label]
        previous = previous_nodes[destination]
        while previous is not None:
            stack.append(previous)
            if previous not in previous_nodes: break
            previous = previous_nodes[previous]

        path = []
        while len(stack) > 0:
            path.append(stack.pop())
        print(path)


    """Ejercicio 05: Camino corto entre dos vertices (fin)"""

    """Ejercicio 07: Ciclo hamiltoniano mas corto (inicio)"""
    def dijkstra_hamiltoniano(self, source, destination):
        distances = {}

        for node in self.nodes:
            distances[node] = float('inf')
        distances[source] = 0

        previous_nodes = {}
        visited = []
        queue = [(0, source)]

        while len(queue) > 0:
            current = heapq.heappop(queue)[1]
            visited.append(current)

            for edge in self.edges[current]:
                if edge.destination.label in visited:
                    continue
                new_distance = distances[current] + edge.weight
                if new_distance < distances[edge.destination.label]:
                    distances[edge.destination.label] = new_distance
                    previous_nodes[edge.destination.label] = current
                    queue.append((new_distance, edge.destination.label))

        return self.build_hamiltonian_path(destination, previous_nodes)

    def build_hamiltonian_path(self, destination, previous_nodes):
        stack = [self.nodes[destination].label]
        previous = previous_nodes[destination]
        while previous is not None:
            stack.append(previous)
            if previous not in previous_nodes:
                break
            previous = previous_nodes[previous]

        path = []
        while len(stack) > 0:
            path.append(stack.pop())

        # Verificar si el camino es un ciclo hamiltoniano cerrado
        if path[0] == path[-1] and len(set(path)) == len(self.nodes):
            print("Ciclo Hamiltoniano encontrado:", " -> ".join(map(str, path)))
        else:
            print("No se encontró un ciclo Hamiltoniano.")
"""Ejercicio 07: Ciclo hamiltoniano mas corto (fin)"""




graph = Grafo()
graph.agregar_nodo('A')
graph.agregar_nodo('B')
graph.agregar_nodo('C')
graph.agregar_nodo('D')
graph.agregar_nodo('E')
graph.agregar_nodo('F')
graph.agregar_nodo('G')
graph.agregar_nodo('H')
graph.agregar_arista('A', 'C')
graph.agregar_arista('C', 'D')
graph.agregar_arista('D', 'B')
graph.agregar_arista('B', 'F')
graph.agregar_arista('F', 'E')
graph.agregar_arista('F', 'G')
graph.agregar_arista('F', 'H')
graph.agregar_arista('H', 'B')
graph.agregar_arista('G', 'E')
graph.agregar_arista('G', 'H')
graph.agregar_arista('E', 'A')


graph.agregar_arista('G', 'A')
graph.agregar_arista('H', 'A')

graph.imprimir_grafo()
print(graph.esta_conectado('D', 'B'))
print(graph.bfs('A'))
print(graph.dfs('A'))
print(graph.componentes('A'))
print(graph.grado_nodo('F'))
print(graph.ciclos_Tarjan('A'))

graph2 = WeightedGraph()
graph2.add_node('A')
graph2.add_node('B')
graph2.add_node('C')
graph2.add_node('D')
graph2.add_node('E')

graph2.add_edge('A', 'C', 5)
graph2.add_edge('A', 'B', 40)
graph2.add_edge('A', 'D', 20)
graph2.add_edge('B', 'D', 10)
graph2.add_edge('B', 'E', 50)
graph2.add_edge('C', 'D', 30)
graph2.add_edge('D', 'E', 100)

graph2.add_edge('C', 'A', 5)
graph2.add_edge('B', 'A', 40)
graph2.add_edge('D', 'A', 20)
graph2.add_edge('D', 'B', 10)
graph2.add_edge('E', 'B', 50)
graph2.add_edge('D', 'C', 30)
graph2.add_edge('E', 'D', 100)

graph2.dijkstra('A', 'E')
#graph2.dijkstra_hamiltoniano('A', 'A')
