use std::collections::{BinaryHeap, HashMap}; // Mengimpor BinaryHeap dan HashMap untuk pengelolaan data.
use std::cmp::Ordering; // Mengimpor enum Ordering untuk perbandingan.
use std::hash::{Hash, Hasher}; // Mengimpor trait Hash dan Hasher untuk hashing custom.
use rand::Rng; // Mengimpor trait Rng untuk menghasilkan angka acak.

// Representasi titik dalam ruang 2D.
#[derive(Debug, Clone, PartialEq)] // Mengimplementasikan Debug, Clone, dan PartialEq.
struct Point {
    x: f64, // Koordinat x.
    y: f64, // Koordinat y.
}

impl Hash for Point {
    fn hash<H: Hasher>(&self, state: &mut H) {
        // Meng-hash nilai x dan y sebagai representasi unik dari Point.
        self.x.to_bits().hash(state);
        self.y.to_bits().hash(state);
    }
}

// Node di graf untuk representasi jalur.
#[derive(Debug, Clone)] // Mengimplementasikan Debug dan Clone.
struct Node {
    id: usize, // ID unik untuk setiap node.
    position: Point, // Posisi node dalam ruang 2D.
    neighbors: Vec<(usize, f64)>, // Daftar tetangga dengan jarak ke tetangga (ID, jarak).
}

// Status dalam algoritma probabilistik.
#[derive(Debug, PartialEq)] // Mengimplementasikan Debug dan PartialEq.
struct State {
    id: usize, // ID node saat ini.
    cost: f64, // Biaya kumulatif dari start ke node ini.
}

impl Eq for State {}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        // Membandingkan biaya dalam urutan menurun agar BinaryHeap memproses yang terendah lebih dulu.
        other.cost.partial_cmp(&self.cost).unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        // Membuat perbandingan parsial dengan fungsi cmp().
        Some(self.cmp(other))
    }
}

// Graf untuk representasi lingkungan robot.
struct Graph {
    nodes: Vec<Node>, // Daftar node dalam graf.
}

impl Graph {
    fn new() -> Self {
        // Membuat graf baru dengan node kosong.
        Graph { nodes: Vec::new() }
    }

    fn add_node(&mut self, id: usize, x: f64, y: f64) {
        // Menambahkan node baru ke graf.
        self.nodes.push(Node {
            id,
            position: Point { x, y }, // Membuat Point dari koordinat.
            neighbors: Vec::new(), // Tetangga awal kosong.
        });
    }

    fn add_edge(&mut self, from: usize, to: usize, distance: f64) {
        // Menambahkan edge antara dua node.
        if let Some(node) = self.nodes.iter_mut().find(|n| n.id == from) {
            node.neighbors.push((to, distance)); // Menambahkan tetangga ke node.
        }
    }

    /// Algoritma pencarian jalur probabilistik.
    fn probabilistic_pathfinding(&self, start_id: usize, goal_id: usize) -> Option<Vec<usize>> {
        let mut heap = BinaryHeap::new(); // Heap untuk memprioritaskan node dengan biaya terendah.
        let mut costs = HashMap::new(); // HashMap untuk menyimpan biaya ke setiap node.
        let mut came_from = HashMap::new(); // HashMap untuk melacak jalur.

        costs.insert(start_id, 0.0); // Biaya awal ke node awal adalah 0.
        heap.push(State { id: start_id, cost: 0.0 }); // Tambahkan node awal ke heap.

        while let Some(State { id, cost }) = heap.pop() {
            // Proses node dengan biaya terendah.
            if id == goal_id {
                // Jika tujuan tercapai, bangun jalur.
                let mut path = Vec::new();
                let mut current = Some(&id);

                while let Some(&node_id) = current {
                    path.push(node_id); // Tambahkan node ke jalur.
                    current = came_from.get(&node_id); // Lanjutkan ke node sebelumnya.
                }

                path.reverse(); // Urutkan jalur dari awal ke tujuan.
                return Some(path); // Kembalikan jalur.
            }

            let current_node = self.nodes.iter().find(|n| n.id == id)?; // Cari node saat ini.
            for &(neighbor_id, distance) in &current_node.neighbors {
                let mut rng = rand::thread_rng(); // Generator angka acak.
                let uncertainty: f64 = rng.gen_range(0.9..1.1); // Faktor ketidakpastian.

                let new_cost = cost + (distance * uncertainty); // Hitung biaya baru dengan ketidakpastian.
                let neighbor_cost = costs.get(&neighbor_id).cloned().unwrap_or(f64::INFINITY);

                if new_cost < neighbor_cost {
                    // Jika biaya baru lebih rendah, perbarui data.
                    costs.insert(neighbor_id, new_cost);
                    came_from.insert(neighbor_id, id);
                    heap.push(State {
                        id: neighbor_id,
                        cost: new_cost,
                    });
                }
            }
        }

        None // Kembalikan None jika tidak ada jalur.
    }
}

fn main() {
    let mut graph = Graph::new(); // Membuat graf kosong.

    // Tambahkan node (ID, x, y).
    graph.add_node(1, 0.0, 0.0);
    graph.add_node(2, 1.0, 1.0);
    graph.add_node(3, 2.0, 2.0);
    graph.add_node(4, 3.0, 1.5);

    // Tambahkan edge (dari, ke, jarak).
    graph.add_edge(1, 2, 1.5);
    graph.add_edge(2, 3, 1.0);
    graph.add_edge(3, 4, 1.2);
    graph.add_edge(1, 3, 2.5);

    let start = 1; // ID node awal.
    let goal = 4; // ID node tujuan.

    match graph.probabilistic_pathfinding(start, goal) {
        Some(path) => println!("Jalur terbaik: {:?}", path), // Tampilkan jalur jika ditemukan.
        None => println!("Tidak ada jalur yang ditemukan."), // Pesan jika jalur tidak ditemukan.
    }
}
