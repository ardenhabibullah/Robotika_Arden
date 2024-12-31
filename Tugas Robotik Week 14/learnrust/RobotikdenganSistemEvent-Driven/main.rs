use std::sync::mpsc::{self, Receiver, Sender}; // Mengimpor modul untuk saluran komunikasi antar-thread.
use std::thread; // Modul untuk bekerja dengan thread.
use std::time::Duration; // Modul untuk mengatur durasi tidur thread.

// Enum untuk berbagai jenis event
enum Event {
    ObstacleDetected(String), // Event ketika robot mendeteksi rintangan.
    GoalChanged(String),      // Event ketika tujuan robot berubah.
    Idle,                     // Event ketika robot dalam keadaan idle (diam).
}

// Struktur untuk Robot
struct Robot {
    name: String, // Nama robot.
}

impl Robot {
    fn new(name: &str) -> Self {
        // Membuat instance baru dari Robot.
        Robot {
            name: name.to_string(),
        }
    }

    // Fungsi untuk menangani event yang diterima
    fn handle_event(&self, event: Event) {
        // Menggunakan `match` untuk menangani berbagai jenis event.
        match event {
            Event::ObstacleDetected(obstacle) => {
                // Jika ada rintangan terdeteksi.
                println!(
                    "🚧 Robot '{}' mendeteksi rintangan: '{}'. Menghitung rute alternatif...",
                    self.name, obstacle
                );
                // Panggil logika untuk menghindari rintangan.
                self.avoid_obstacle();
            }
            Event::GoalChanged(new_goal) => {
                // Jika tujuan berubah.
                println!(
                    "🎯 Robot '{}' memiliki tujuan baru: '{}'. Memulai navigasi...",
                    self.name, new_goal
                );
                // Panggil logika untuk menuju tujuan baru.
                self.navigate_to(&new_goal);
            }
            Event::Idle => {
                // Jika robot idle.
                println!("⏳ Robot '{}' sedang dalam keadaan idle.", self.name);
            }
        }
    }

    fn avoid_obstacle(&self) {
        // Logika untuk menghindari rintangan.
        println!("🤖 Robot '{}' menghindari rintangan...", self.name);
        thread::sleep(Duration::from_secs(1)); // Tidur selama 1 detik untuk mensimulasikan waktu proses.
        println!("✅ Rintangan berhasil dihindari.");
    }

    fn navigate_to(&self, goal: &str) {
        // Logika untuk navigasi ke tujuan baru.
        println!("🛤️ Robot '{}' bergerak menuju tujuan: '{}'", self.name, goal);
        thread::sleep(Duration::from_secs(2)); // Tidur selama 2 detik untuk mensimulasikan waktu perjalanan.
        println!("🏁 Robot '{}' telah mencapai tujuan: '{}'", self.name, goal);
    }
}

// Fungsi untuk memicu event secara acak (simulasi sensor lingkungan).
fn environment_simulation(sender: Sender<Event>) {
    let events = vec![
        // Daftar event yang akan dikirimkan ke robot.
        Event::ObstacleDetected(String::from("Batu Besar")),
        Event::GoalChanged(String::from("Pos A")),
        Event::Idle,
        Event::GoalChanged(String::from("Pos B")),
        Event::ObstacleDetected(String::from("Parit Lebar")),
    ];

    for event in events {
        println!("\n🌍 Event baru terdeteksi di lingkungan."); // Notifikasi event baru.
        sender.send(event).expect("Gagal mengirim event"); // Mengirim event ke robot melalui saluran komunikasi.
        thread::sleep(Duration::from_secs(3)); // Tidur selama 3 detik sebelum mengirim event berikutnya.
    }
}

// Fungsi utama
fn main() {
    // Membuat saluran komunikasi untuk event.
    let (tx, rx): (Sender<Event>, Receiver<Event>) = mpsc::channel();

    let robot = Robot::new("Atlas"); // Membuat robot dengan nama "Atlas".

    // Simulasi lingkungan berjalan di thread terpisah.
    let env_thread = thread::spawn(move || {
        environment_simulation(tx); // Menjalankan simulasi lingkungan.
    });

    // Event loop untuk menangani event.
    for event in rx {
        robot.handle_event(event); // Robot menangani setiap event yang diterima.
    }

    env_thread.join().expect("Thread simulasi lingkungan gagal selesai"); // Menunggu thread simulasi selesai.
}
