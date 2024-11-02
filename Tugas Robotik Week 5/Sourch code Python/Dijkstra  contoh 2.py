__author__ = 'blkrt'
import networkx as nx
import matplotlib.pyplot as plt

def backtrace(parent, start, end):
    path = [end]
    while path[-1] != start:
        path.append(parent[path[-1]])
    path.reverse()
    return path

def dijkstra(graph, source, target):
    queue = []
    visited = {}
    distance = {}
    parent = {}

    # Inisialisasi jarak dan status kunjungan untuk semua node
    for node in graph.nodes:
        distance[node] = float("inf")
        visited[node] = False
        parent[node] = None

    distance[source] = 0
    queue.append((0, source))  # Priority queue

    while queue:
        # Pilih node dengan jarak terkecil
        queue = sorted(queue, key=lambda x: x[0])  # Sort queue by distance
        current_distance, current = queue.pop(0)
        visited[current] = True

        # Jika mencapai target, hitung jalur dan biaya total
        if current == target:
            path = backtrace(parent, source, target)
            print("Shortest Path:", path)
            print("Total Cost:", distance[target])
            return path, distance[target]

        # Update jarak untuk setiap tetangga
        for neighbor in graph.neighbors(current):
            weight = graph[current][neighbor]['weight']
            if not visited[neighbor]:
                new_distance = current_distance + weight
                if new_distance < distance[neighbor]:
                    distance[neighbor] = new_distance
                    parent[neighbor] = current
                    queue.append((new_distance, neighbor))

    return [], float("inf")  # Jika path tidak ditemukan

def main():
    # Membuat graf yang lebih kompleks dengan beberapa node dan bobot edge
    G = nx.Graph()
    edges = [
        (0, 1, 2), (1, 2, 3), (0, 3, 1), (3, 4, 4), (4, 5, 2), 
        (5, 6, 1), (1, 6, 3), (2, 5, 2), (2, 7, 1), (6, 7, 2),
        (3, 7, 4), (4, 7, 3)
    ]
    G.add_weighted_edges_from(edges)

    # Menjalankan Dijkstra dari node 0 ke node 7 dan mendapatkan jalur dan biaya total
    path, total_cost = dijkstra(G, 0, 7)

    # Menampilkan graf dan jalur terpendek
    pos = nx.spring_layout(G)  # Posisi semua node
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=500, font_size=10)
    labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)

    # Menyoroti jalur terpendek dalam graf
    path_edges = list(zip(path, path[1:]))  # Membuat edge dari node di jalur
    nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='red', width=2.5)

    plt.title(f"Shortest Path from 0 to 7 with Total Cost: {total_cost}")
    plt.show()

if __name__ == "__main__":
    main()
