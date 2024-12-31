use std::collections::{BinaryHeap, HashMap}; // Mengimpor BinaryHeap untuk prioritas antrian dan HashMap untuk peta data.
use std::cmp::Ordering; // Mengimpor trait untuk perbandingan.

// Struktur Node merepresentasikan simpul dalam grid.
#[derive(Copy, Clone, Eq, PartialEq)]
struct Node {
    position: (usize, usize), // Posisi node dalam koordinat grid.
    cost: usize,             // Biaya aktual dari titik awal.
    heuristic: usize,        // Estimasi biaya ke tujuan (heuristik).
}

impl Node {
    fn f(&self) -> usize {
        // Fungsi f(n) = g(n) + h(n), yaitu biaya aktual ditambah heuristik.
        self.cost + self.heuristic
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        // Membandingkan node berdasarkan nilai f(), diurutkan secara terbalik (untuk BinaryHeap).
        other.f().cmp(&self.f())
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        // Partial comparison menggunakan implementasi `cmp`.
        Some(self.cmp(other))
    }
}

// Fungsi untuk menghitung heuristik menggunakan jarak Manhattan.
fn heuristic(a: (usize, usize), b: (usize, usize)) -> usize {
    a.0.abs_diff(b.0) + a.1.abs_diff(b.1) // Menggunakan perbedaan absolut koordinat.
}

// Implementasi algoritma A*.
fn a_star(grid: &Vec<Vec<usize>>, start: (usize, usize), goal: (usize, usize)) -> Option<Vec<(usize, usize)>> {
    let mut open_set = BinaryHeap::new(); // Antrian prioritas untuk node yang akan diperiksa.
    let mut came_from: HashMap<(usize, usize), (usize, usize)> = HashMap::new(); // Untuk melacak jalur yang diambil.
    let mut g_score: HashMap<(usize, usize), usize> = HashMap::new(); // Menyimpan biaya aktual dari titik awal.

    g_score.insert(start, 0); // Biaya awal ke node awal adalah 0.
    open_set.push(Node {
        position: start,
        cost: 0,
        heuristic: heuristic(start, goal), // Hitung heuristik awal.
    });

    let directions = vec![
        (1, 0),  // Ke bawah.
        (0, 1),  // Ke kanan.
        (-1, 0), // Ke atas.
        (0, -1), // Ke kiri.
    ];

    while let Some(current) = open_set.pop() {
        // Ambil node dengan prioritas tertinggi.
        if current.position == goal {
            // Jika node saat ini adalah tujuan, bangun jalur.
            let mut path = vec![goal];
            let mut curr = goal;
            while let Some(&prev) = came_from.get(&curr) {
                path.push(prev);
                curr = prev;
            }
            path.reverse(); // Balikkan jalur untuk urutan dari awal ke tujuan.
            return Some(path);
        }

        for (dx, dy) in &directions {
            // Periksa semua tetangga (atas, bawah, kiri, kanan).
            let (x, y) = (current.position.0 as isize + dx, current.position.1 as isize + dy);
            if x < 0 || y < 0 || x >= grid.len() as isize || y >= grid[0].len() as isize {
                // Jika keluar dari grid, abaikan.
                continue;
            }
            let neighbor = (x as usize, y as usize);
            if grid[neighbor.0][neighbor.1] == 1 {
                // Jika tetangga adalah rintangan, abaikan.
                continue;
            }
            let tentative_g_score = g_score[&current.position] + 1; // Hitung biaya baru.
            if tentative_g_score < *g_score.get(&neighbor).unwrap_or(&usize::MAX) {
                // Jika biaya baru lebih kecil, perbarui nilai.
                came_from.insert(neighbor, current.position);
                g_score.insert(neighbor, tentative_g_score);
                open_set.push(Node {
                    position: neighbor,
                    cost: tentative_g_score,
                    heuristic: heuristic(neighbor, goal), // Perbarui heuristik tetangga.
                });
            }
        }
    }
    None // Jika tidak ada jalur ke tujuan ditemukan.
}

fn main() {
    let grid = vec![
        vec![0, 0, 0, 0, 0], // 0 menunjukkan ruang bebas.
        vec![0, 1, 1, 1, 0], // 1 menunjukkan rintangan.
        vec![0, 1, 0, 1, 0],
        vec![0, 1, 0, 1, 0],
        vec![0, 0, 0, 0, 0],
    ];

    let start = (0, 0); // Titik awal.
    let goal = (4, 4); // Titik tujuan.

    match a_star(&grid, start, goal) {
        Some(path) => {
            println!("Jalur ditemukan:");
            for (i, (x, y)) in path.iter().enumerate() {
                println!("Langkah {}: ({}, {})", i + 1, x, y); // Tampilkan setiap langkah dalam jalur.
            }
        }
        None => println!("Tidak ada jalur yang ditemukan."), // Jika tidak ada jalur.
    }
}
