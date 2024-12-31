use std::collections::{BinaryHeap, HashMap}; // Mengimpor koleksi BinaryHeap dan HashMap untuk digunakan.
use std::cmp::Ordering; // Mengimpor enum Ordering untuk membandingkan nilai.

#[derive(Copy, Clone, Eq, PartialEq)] // Mengimplementasikan sifat copyable dan equatable untuk struktur Node.
struct Node {
    position: (usize, usize), // Posisi node dalam grid (baris, kolom).
    cost: usize,              // g(n): Biaya kumulatif dari awal ke node ini.
    heuristic: usize,         // h(n): Estimasi biaya dari node ini ke tujuan.
}

impl Node {
    fn f(&self) -> usize {
        self.cost + self.heuristic // Menghitung fungsi f(n) = g(n) + h(n).
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        other.f().cmp(&self.f()) // Membandingkan node berdasarkan nilai f(n), lebih kecil lebih prioritas.
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other)) // Implementasi parsial untuk perbandingan.
    }
}

// Fungsi untuk menghitung jarak Manhattan antara dua titik.
fn heuristic(a: (usize, usize), b: (usize, usize)) -> usize {
    a.0.abs_diff(b.0) + a.1.abs_diff(b.1) // |x1 - x2| + |y1 - y2|.
}

// Implementasi algoritma A* untuk menemukan jalur.
fn a_star(grid: &Vec<Vec<usize>>, start: (usize, usize), goal: (usize, usize)) -> Option<Vec<(usize, usize)>> {
    let mut open_set = BinaryHeap::new(); // Heap untuk node yang akan dievaluasi.
    let mut came_from: HashMap<(usize, usize), (usize, usize)> = HashMap::new(); // Menyimpan jalur yang dilalui.
    let mut g_score: HashMap<(usize, usize), usize> = HashMap::new(); // Menyimpan nilai g(n) untuk setiap node.

    g_score.insert(start, 0); // Inisialisasi g(n) untuk node awal.
    open_set.push(Node {
        position: start,
        cost: 0,
        heuristic: heuristic(start, goal), // h(n) dihitung berdasarkan fungsi heuristic.
    });

    // Arah gerakan (kanan, bawah, kiri, atas).
    let directions = vec![
        (1, 0),  // Gerak ke bawah.
        (0, 1),  // Gerak ke kanan.
        (-1, 0), // Gerak ke atas.
        (0, -1), // Gerak ke kiri.
    ];

    // Proses algoritma A*.
    while let Some(current) = open_set.pop() { // Mengambil node dengan nilai f(n) terkecil.
        if current.position == goal { // Jika node tujuan tercapai.
            let mut path = vec![goal]; // Inisialisasi jalur.
            let mut curr = goal;
            while let Some(&prev) = came_from.get(&curr) { // Menelusuri jalur balik.
                path.push(prev);
                curr = prev;
            }
            path.reverse(); // Membalik jalur agar dimulai dari start ke goal.
            return Some(path); // Mengembalikan jalur yang ditemukan.
        }

        for (dx, dy) in &directions { // Iterasi semua arah gerakan.
            let (x, y) = (current.position.0 as isize + dx, current.position.1 as isize + dy);
            if x < 0 || y < 0 || x >= grid.len() as isize || y >= grid[0].len() as isize {
                continue; // Lewati jika di luar batas grid.
            }
            let neighbor = (x as usize, y as usize);
            if grid[neighbor.0][neighbor.1] == 1 {
                continue; // Lewati jika merupakan rintangan.
            }
            let tentative_g_score = g_score[&current.position] + 1; // Hitung g(n) sementara.
            if tentative_g_score < *g_score.get(&neighbor).unwrap_or(&usize::MAX) {
                came_from.insert(neighbor, current.position); // Simpan jalur balik.
                g_score.insert(neighbor, tentative_g_score); // Perbarui g(n) node tetangga.
                open_set.push(Node {
                    position: neighbor,
                    cost: tentative_g_score,
                    heuristic: heuristic(neighbor, goal), // Hitung nilai f(n).
                });
            }
        }
    }
    None // Jika tidak ditemukan jalur.
}

fn main() {
    let grid = vec![ // Peta grid: 0 = jalan, 1 = rintangan.
        vec![0, 0, 0, 0, 0],
        vec![0, 1, 1, 1, 0],
        vec![0, 1, 0, 1, 0],
        vec![0, 1, 0, 1, 0],
        vec![0, 0, 0, 0, 0],
    ];

    let start = (0, 0); // Posisi awal.
    let goal = (4, 4); // Posisi tujuan.

    match a_star(&grid, start, goal) { // Jalankan algoritma A*.
        Some(path) => { // Jika jalur ditemukan.
            println!("Jalur ditemukan:");
            for (x, y) in path {
                println!("({}, {})", x, y); // Cetak setiap node dalam jalur.
            }
        }
        None => println!("Tidak ada jalur yang ditemukan."), // Jika jalur tidak ditemukan.
    }
}
