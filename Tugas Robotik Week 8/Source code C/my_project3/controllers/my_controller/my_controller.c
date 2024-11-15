#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>

#define TIME_STEP 64

int main(int argc, char **argv) {
  wb_robot_init();

  // Mengambil perangkat motor dan sensor
  WbDeviceTag motor_kanan = wb_robot_get_device("right wheel motor");
  WbDeviceTag motor_kiri = wb_robot_get_device("left wheel motor");
  WbDeviceTag sensor_kanan = wb_robot_get_device("right_sensor");
  WbDeviceTag sensor_kiri = wb_robot_get_device("left_sensor");

  // Cek apakah perangkat ditemukan
  if (sensor_kanan == 0 || sensor_kiri == 0) {
    printf("Error: One or both sensors not found. Check device names.\n");
    wb_robot_cleanup();
    return -1;
  }

  // Mengaktifkan sensor dengan interval waktu yang telah ditentukan
  wb_distance_sensor_enable(sensor_kanan, TIME_STEP);
  wb_distance_sensor_enable(sensor_kiri, TIME_STEP);
  
  // Menetapkan motor agar berputar terus-menerus tanpa batas
  wb_motor_set_position(motor_kanan, INFINITY);
  wb_motor_set_position(motor_kiri, INFINITY);

  // Variabel untuk menyimpan nilai sensor
  double readIR_kanan, readIR_kiri;

  // Loop utama simulasi robot
  while (wb_robot_step(TIME_STEP) != -1) {
    // Membaca nilai sensor kanan dan kiri
    readIR_kanan = wb_distance_sensor_get_value(sensor_kanan);
    readIR_kiri = wb_distance_sensor_get_value(sensor_kiri);
    
    // Menampilkan nilai sensor ke layar untuk debugging
    printf("kanan: %f\n", readIR_kanan);
    printf("kiri: %f\n", readIR_kiri);

    // Logika gerakan berdasarkan perbandingan sensor kanan dan kiri
    if (readIR_kiri < readIR_kanan) {
      wb_motor_set_velocity(motor_kanan, 1.0);  // Kecepatan motor kanan
      wb_motor_set_velocity(motor_kiri, 2.5);   // Kecepatan motor kiri
    }
    
    if (readIR_kiri > readIR_kanan) {
      wb_motor_set_velocity(motor_kanan, 2.5);  // Kecepatan motor kanan
      wb_motor_set_velocity(motor_kiri, 1.0);   // Kecepatan motor kiri
    }
  }

  wb_robot_cleanup();  // Membersihkan dan menutup robot setelah simulasi selesai
  return 0;
}
