import networkx as nx
import matplotlib.pyplot as plt
from geopy.distance import geodesic
from scipy.spatial import cKDTree


min_distance=0.2
nodes = [
(37.325124,30.239592),
(37.3046917,31.7329934),
(37.2909849,31.7654321),
(37.289181,30.243952),
(37.289134,30.244189),
(37.286678,30.251215),
(37.27331,31.7794117),
(37.272811,30.291861),
(37.259225,30.403241),
(37.249135,30.236406),
(37.2475181,30.2258172),
(37.247274,30.401863),
(37.24149,30.358899),
(37.223897,30.406736),

]

G = nx.Graph()

for i, node in enumerate(nodes):
    G.add_node(i, pos=node)

for i in range(len(nodes)):
    for j in range(i + 1, len(nodes)):
        distance = geodesic(nodes[i], nodes[j]).km
        if distance<=min_distance:  # umbral de distancia para agregar aristas
            G.add_edge(i, j, weight=distance)

pos = {i: node for i, node in enumerate(nodes)}

# Dibujamos el grafo original
#nx.draw(G, pos, with_labels=True)
#plt.show()

# Obtenemos los nodos vecinos más cercanos
kdtree = cKDTree(nodes)
for i in range(len(nodes)):
    distances, indices = kdtree.query(nodes[i], k=4)
    neighbors = list(indices[1:])
    for neighbor in neighbors:
        distance = geodesic(nodes[i], nodes[neighbor]).km
        if distance<=min_distance:  # umbral de distancia para agregar aristas
            G.add_edge(i, neighbor, weight=distance)

# Dibujamos el grafo con las nuevas aristas
edge_labels = {(u, v): round(d['weight'], 2) for u, v, d in G.edges(data=True)}

nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels,font_size=8)
nx.draw(G, pos, with_labels=False,node_size=10, width=1)

plt.show()
