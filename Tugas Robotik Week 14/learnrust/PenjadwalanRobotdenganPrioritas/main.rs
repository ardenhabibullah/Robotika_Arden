use std::collections::BinaryHeap; // Mengimpor BinaryHeap untuk membuat antrean prioritas.
use std::cmp::Ordering; // Mengimpor enum Ordering untuk mengimplementasikan perbandingan.

// Struktur untuk mewakili tugas dengan prioritas.
#[derive(Eq, PartialEq)] // Mengimplementasikan Eq dan PartialEq untuk perbandingan tugas.
struct Task {
    name: String,       // Nama tugas.
    priority: u8,       // Prioritas tugas (semakin besar semakin tinggi).
    description: String, // Deskripsi tugas.
}

// Implementasi trait Ord untuk Task agar bisa diurutkan dalam antrean prioritas.
impl Ord for Task {
    fn cmp(&self, other: &Self) -> Ordering {
        // Membalik urutan agar tugas dengan prioritas lebih tinggi diproses lebih dahulu.
        other.priority.cmp(&self.priority)
    }
}

impl PartialOrd for Task {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other)) // Mengimplementasikan perbandingan parsial dengan cmp().
    }
}

// Enum untuk status robot.
enum RobotState {
    Idle, // Status robot sedang tidak bekerja.
    Busy, // Status robot sedang memproses tugas.
}

// Struktur untuk merepresentasikan robot.
struct Robot {
    name: String,              // Nama robot.
    state: RobotState,         // Status robot saat ini (Idle atau Busy).
    task_queue: BinaryHeap<Task>, // Antrean prioritas untuk tugas-tugas robot.
}

impl Robot {
    // Fungsi untuk inisialisasi robot baru.
    fn new(name: &str) -> Self {
        Robot {
            name: name.to_string(), // Mengatur nama robot.
            state: RobotState::Idle, // Inisialisasi status robot sebagai Idle.
            task_queue: BinaryHeap::new(), // Membuat antrean tugas kosong.
        }
    }

    // Fungsi untuk menambahkan tugas ke antrean prioritas.
    fn add_task(&mut self, name: &str, priority: u8, description: &str) {
        self.task_queue.push(Task {
            name: name.to_string(), // Nama tugas.
            priority,               // Prioritas tugas.
            description: description.to_string(), // Deskripsi tugas.
        });
        // Memberi informasi bahwa tugas berhasil ditambahkan ke antrean.
        println!("Tugas '{}' dengan prioritas {} ditambahkan ke antrean.", name, priority);
    }

    // Fungsi untuk memproses tugas dari antrean prioritas.
    fn process_tasks(&mut self) {
        while let Some(task) = self.task_queue.pop() { // Mengambil tugas dengan prioritas tertinggi.
            self.state = RobotState::Busy; // Mengatur status robot menjadi Busy.
            println!(
                "🚀 Robot '{}' sedang memproses tugas: '{}' (Prioritas: {}) - {}",
                self.name, task.name, task.priority, task.description
            );
            // Simulasi waktu pemrosesan tugas (1 detik per tugas).
            std::thread::sleep(std::time::Duration::from_secs(1));
        }
        self.state = RobotState::Idle; // Mengatur status robot kembali menjadi Idle.
        println!("✅ Semua tugas selesai. Robot '{}' kembali ke status Idle.", self.name);
    }
}

// Fungsi utama program.
fn main() {
    let mut robot = Robot::new("Atlas"); // Membuat robot baru dengan nama "Atlas".

    // Menambahkan beberapa tugas ke antrean.
    robot.add_task("Angkut Barang", 2, "Mengangkut barang ke gudang.");
    robot.add_task("Isi Ulang Baterai", 1, "Mengisi ulang baterai robot.");
    robot.add_task("Perbaikan Mesin", 3, "Memperbaiki mesin produksi.");

    // Memproses semua tugas berdasarkan prioritas (tugas prioritas lebih tinggi diproses lebih dahulu).
    robot.process_tasks();
}

