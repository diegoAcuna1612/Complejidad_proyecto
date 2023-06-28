import csv
import math
import networkx as nx
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
import pandas as pd
import tkinter as tk
from PIL import Image, ImageTk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

discoruta = 'C:/TF/new_stop2.csv'

def calculate_distance(node1, node2):
    lat1, lon1 = node1
    lat2, lon2 = node2
    dx = lat2 - lat1
    dy = lon2 - lon1
    return math.sqrt(dx ** 2 + dy ** 2)

def build_graph(nodes):
    graph = nx.Graph()
    for i, node in enumerate(nodes):
        graph.add_node(node[0])
        for j, other_node in enumerate(nodes):
            if i != j:
                distance = calculate_distance(node[2:], other_node[2:])
                if distance < 0.2:
                    weight = round(distance * 1000, 5)  # Convertir distancia a metros con cinco decimales
                    graph.add_edge(node[0], other_node[0], weight=weight)
                    graph.edges[node[0], other_node[0]]['weight'] = weight
    return graph

def dijkstra(graph, start_node, end_node):
    global shortest_path, shortest_distance
    shortest_path = nx.dijkstra_path(graph, start_node, end_node)
    shortest_distance = nx.dijkstra_path_length(graph, start_node, end_node)
    return shortest_path, shortest_distance

nodes = []

with open(f'{discoruta}', newline='') as csvfile:
    reader = csv.reader(csvfile)
    next(reader)  # Salta la primera línea de encabezados
    for row in reader:
        stop_name = row[0]
        stop_id = row[1]
        latitude = float(row[2])
        longitude = float(row[3])
        nodes.append((stop_name, stop_id, latitude, longitude))

data = []

with open(f'{discoruta}', 'r') as file:
    csv_reader = csv.reader(file)
    next(csv_reader)  # Salta la primera línea de encabezados
    for row in csv_reader:
        data.append(row[0])  # datos de la 1era columna

#---------------------------------------------------------#

def on_dropdown_select(event):
    global parada1,parada2
    parada1 = dropdown1.get()
    parada2 = dropdown2.get()

def submit():
    global ingnodo
    ingnodo = input3.get()
    window.destroy()

ruta11='c:/TF/ant.jpg'

window = tk.Tk()

bgal='gray'
white='white'

window.geometry("300x400")
window.title("Complejidad Algoritmica")
window.configure(bg=bgal)

title_label = tk.Label(window, text="Antalya Rutas", font=("Arial", 24),fg=white,bg=bgal)
title_label.pack(side="top", pady=15)

frame = tk.Frame(window)
frame.pack(expand=True)

input_frame = tk.Frame(window, bg=bgal)
input_frame.pack(expand=True)

# listas parada 1
dropdown_label1 = tk.Label(window, text="Parada 1:",font=("Arial", 16),pady=5,fg=white,bg=bgal)
dropdown_label1.pack()

dropdown1 = ttk.Combobox(window, values=data, state="readonly")
dropdown1.bind("<<ComboboxSelected>>", on_dropdown_select)
dropdown1.pack()

# listas parada 2
dropdown_label2 = tk.Label(window, text="Parada 2:",font=("Arial", 16),pady=5,fg=white,bg=bgal)
dropdown_label2.pack()

dropdown2 = ttk.Combobox(window, values=data, state="readonly")
dropdown2.bind("<<ComboboxSelected>>", on_dropdown_select)
dropdown2.pack()

# nodos vecinos
label3 = tk.Label(input_frame, text="Nro de Nodos",font=("Arial", 16),pady=5,fg=white,bg=bgal)
label3.pack()
input3 = tk.Entry(input_frame,width=15)
input3.pack(pady=15)

#generar grafo y calculos
submit_button = tk.Button(window, text="Generar", command=submit,width=15,font=("Arial", 16))
submit_button.pack(pady=40)

window.mainloop()

#---------------------------------------------------------#

start_node_name = parada1
end_node_name = parada2
num_neighbors = int(ingnodo)

def antalya():

    graph = build_graph(nodes)

    # Obtener el índice del nodo de origen
    start_node_index = None
    for i, node in enumerate(nodes):
        if node[0] == start_node_name:
            start_node_index = i
            break
    # Crear una lista de puntos de nodos para construir el árbol cKDTree
    node_points = [(node[2], node[3]) for node in nodes]

    # Obtenemos los nodos vecinos más cercanos
    kdtree = cKDTree(node_points)
    distances, indices = kdtree.query(node_points[start_node_index], k=num_neighbors + 1)  # Obtenemos los nodos más cercanos
    nearest_neighbor_indices = indices[1:]  # Excluimos el primer índice, que corresponde al nodo de origen

    # Obtener los nombres de los nodos vecinos más cercanos
    nearest_neighbor_names = [nodes[i][0] for i in nearest_neighbor_indices]

    # Conectar los nodos vecinos más cercanos con el nodo de origen y destino
    for neighbor_index in nearest_neighbor_indices:
        if neighbor_index < len(nodes):
            graph.add_edge(start_node_name, nodes[neighbor_index][0], weight=calculate_distance(nodes[start_node_index][2:], nodes[neighbor_index][2:]))

    # Calcular las rutas más cortas desde el punto de origen a cada uno de los nodos vecinos más cercanos
    shortest_paths_from_start = {}
    for neighbor_name in nearest_neighbor_names:
        shortest_path, shortest_distance = dijkstra(graph, start_node_name, neighbor_name)
        shortest_paths_from_start[neighbor_name] = (shortest_path, shortest_distance)

    # Calcular las rutas más cortas desde cada uno de los nodos vecinos más cercanos al punto de destino
    shortest_paths_to_end = {}
    for neighbor_name in nearest_neighbor_names:
        shortest_path, shortest_distance = dijkstra(graph, neighbor_name, end_node_name)
        shortest_paths_to_end[neighbor_name] = (shortest_path, shortest_distance)

    # Encontrar la ruta más corta entre todas las combinaciones posibles y combinar las rutas correspondientes
    shortest_path = None
    shortest_distance = float('inf')
    for neighbor_name in nearest_neighbor_names:
        path_start_to_neighbor, distance_start_to_neighbor = shortest_paths_from_start[neighbor_name]
        path_neighbor_to_end, distance_neighbor_to_end = shortest_paths_to_end[neighbor_name]
        total_distance = distance_start_to_neighbor + distance_neighbor_to_end
        if total_distance < shortest_distance:
            shortest_distance = total_distance
            shortest_path = path_start_to_neighbor + path_neighbor_to_end[1:]  # Excluimos el primer nodo del camino desde el vecino hasta el destino

    # Crear subgrafo con los nodos de origen, destino y vecinos más cercanos
    subgraph_nodes = [start_node_name] + nearest_neighbor_names + [end_node_name]
    subgraph = graph.subgraph(subgraph_nodes)

    # Obtener los bordes de la ruta más corta entre los vecinos más cercanos
    shortest_path_edges_nearest = list(zip(shortest_path, shortest_path[1:]))

    # Dibujar el subgrafo con etiquetas de distancia
    pos = nx.spring_layout(subgraph, k=0.2, scale=0.5)
    edge_labels = nx.get_edge_attributes(subgraph, 'weight')
    node_labels = {node: node for node in subgraph.nodes()}
    node_colors = ['lightblue' if node != start_node_name and node != end_node_name else 'orange' for node in subgraph.nodes()]

    # Definir las características de las aristas
    edge_widths = [2.0 if (u, v) in shortest_path_edges_nearest or (v, u) in shortest_path_edges_nearest else 0.5 for u, v in subgraph.edges()]
    edge_colors = ['blue' if (u, v) in shortest_path_edges_nearest or (v, u) in shortest_path_edges_nearest else 'gray' for u, v in subgraph.edges()]

    # Obtener las coordenadas de latitud y longitud de los nodos
    node_positions = {node[0]: (node[3], node[2]) for node in nodes}

    # Dibujar el grafo con las coordenadas reales
    plt.figure(figsize=(8, 8))
    nx.draw_networkx_nodes(subgraph, node_positions, node_color=node_colors, node_size=500, alpha=0.8)
    nx.draw_networkx_edges(subgraph, node_positions, edge_color=edge_colors, width=edge_widths, alpha=0.7)
    nx.draw_networkx_labels(subgraph, node_positions, labels=node_labels, font_size=8, font_color='black', verticalalignment='center')

    # Modificar la etiqueta de la arista para mostrar los metros como enteros
    edge_labels_rounded = {edge: f"{int(weight)}m" for edge, weight in edge_labels.items()}  # Modificar la etiqueta para mostrar enteros
    nx.draw_networkx_edge_labels(subgraph, pos, edge_labels=edge_labels_rounded, font_size=10, label_pos=0.5, rotate=False,
                                 font_color='black', bbox=dict(facecolor='white', edgecolor='none', boxstyle='round,pad=0.2'))

    # Agregar etiquetas de distancia entre nodos
    for u, v, weight in subgraph.edges(data='weight'):
        x = (node_positions[u][0] + node_positions[v][0]) / 2
        y = (node_positions[u][1] + node_positions[v][1]) / 2
        plt.text(x, y, f"{int(weight)}m", ha='center', va='center', fontsize=8)  # Modificar la etiqueta para mostrar enteros

    plt.title("Grafo desde el paradero {} hasta los nodos vecinos más cercanos y el paradero de destino".format(start_node_name))
    plt.tight_layout()
    plt.show()

    return shortest_path, shortest_distance

def extrainfo():
    antalya()

    color1 = "Black"
    color2 = "White"

    new_window = tk.Tk()
    new_window.geometry("900x300")
    new_window.title("Resultados")
    new_window.configure(background=color1)    

    canvas1 = tk.Canvas(new_window, width=900, height=300, bg=color1)
    canvas1.pack(fill="both", expand=True)

    new_label1 = canvas1.create_text(450, 50, text="Ruta más corta:", font=("Arial", 14), fill=color2)
    new_label2 = canvas1.create_text(450, 100, text=f"{shortest_path}", font=("Arial", 14), fill=color2)
    new_label3 = canvas1.create_text(450, 170, text="Distancia más corta:", font=("Arial", 14), fill=color2)
    new_label4 = canvas1.create_text(450, 220, text=f"{round(shortest_distance, 4)} km", font=("Arial", 14), fill=color2)

    new_window.mainloop()

extrainfo()

