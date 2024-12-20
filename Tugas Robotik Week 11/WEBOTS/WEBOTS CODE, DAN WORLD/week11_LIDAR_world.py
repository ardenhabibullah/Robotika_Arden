from controller import Robot

# Konstanta waktu langkah simulasi
TIME_STEP = 32

# Indeks untuk sensor kiri dan kanan
LEFT = 0
RIGHT = 1

# Inisialisasi robot
robot = Robot()

# Inisialisasi lidar
lidar = robot.getDevice('lidar')   # Lidar pertama
lidar1 = robot.getDevice('lidar1')  # Lidar kedua
lidar2 = robot.getDevice('lidar2')  # Lidar ketiga
lidar3 = robot.getDevice('lidar3')  # Lidar keempat, pastikan nama perangkat sesuai dengan simulasi

lidar.enable(TIME_STEP)
lidar.enablePointCloud()

lidar1.enable(TIME_STEP)  # Mengaktifkan lidar kedua
lidar1.enablePointCloud()

lidar2.enable(TIME_STEP)  # Mengaktifkan lidar ketiga
lidar2.enablePointCloud()

lidar3.enable(TIME_STEP)  # Mengaktifkan lidar keempat
lidar3.enablePointCloud()

# Inisialisasi sensor jarak (ultrasonic)
us = [robot.getDevice('ps0'), robot.getDevice('ps1')]  # Sensor jarak
for sensor in us:
    sensor.enable(TIME_STEP)

# Inisialisasi motor
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))  # Aktifkan mode kecepatan
right_motor.setPosition(float('inf'))  # Aktifkan mode kecepatan
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Koefisien empiris untuk penghindaran tabrakan
coefficients = [[12.0, -6.0], [-10.0, 8.0]]
base_speed = 3.0

# Fungsi untuk membaca data lidar
def extract_lidar_data():
    lidar_data = lidar.getRangeImage()
    lidar1_data = lidar1.getRangeImage()  # Data dari lidar1
    lidar2_data = lidar2.getRangeImage()  # Data dari lidar2
    lidar3_data = lidar3.getRangeImage()  # Data dari lidar3
    print(f'Lidar Data 1: {lidar_data[:10]}...')  # Menampilkan 10 data pertama dari lidar
    print(f'Lidar Data 2: {lidar1_data[:10]}...')  # Menampilkan 10 data pertama dari lidar1
    print(f'Lidar Data 3: {lidar2_data[:10]}...')  # Menampilkan 10 data pertama dari lidar2
    print(f'Lidar Data 4: {lidar3_data[:10]}...')  # Menampilkan 10 data pertama dari lidar3
    print(f'Lidar 3 Raw Data: {lidar3_data}')  # Menampilkan data lidar3 secara keseluruhan
    return lidar_data, lidar1_data, lidar2_data, lidar3_data  # Kembalikan data dari keempat lidar

# Fungsi untuk membaca data dari sensor jarak
def read_distance_sensors():
    distances = [sensor.getValue() for sensor in us]
    print(f'Distance Sensor Readings Left={distances[LEFT]:.2f}, Right={distances[RIGHT]:.2f}')
    return distances

# Fungsi untuk menghitung kecepatan berdasarkan data sensor
def compute_speeds(us_values):
    speed = [0.0, 0.0]
    for i in range(2):
        for k in range(2):
            speed[i] += us_values[k] * coefficients[i][k]  # Perbaiki sintaks perkalian
    return speed

# Loop utama
while robot.step(TIME_STEP) != -1:
    # Baca data lidar dan ekstrak informasi
    lidar_data, lidar1_data, lidar2_data, lidar3_data = extract_lidar_data()

    # Baca data sensor jarak
    us_values = read_distance_sensors()

    # Hitung kecepatan roda berdasarkan data sensor
    speeds = compute_speeds(us_values)

    # Atur kecepatan motor
    left_motor.setVelocity(base_speed + (speeds[LEFT]/300))
    right_motor.setVelocity(base_speed + (speeds[RIGHT]/300))

# Membersihkan memori setelah simulasi selesai
robot.cleanup()
