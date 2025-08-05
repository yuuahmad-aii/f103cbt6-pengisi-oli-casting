# STM32F103CBT6 Pengisi Oli Casting

## Deskripsi
Sistem otomatis pengisian oli berbasis STM32F103CBT6 untuk tiga drum (A, B, C) dengan sensor level ultrasonik (protokol 0xA0, 24-bit), kontrol prioritas, histeresis, proteksi sumber kosong, dan antarmuka CLI via USB VCP. Mendukung LCD I2C dan SD Card (opsional).

## Fitur Utama
- Kontrol otomatis pengisian drum dengan prioritas (A > B > C)
- Sensor level via UART (protokol 0xA0, 3-byte data, opsional checksum)
- Histeresis pengisian (start/stop berdasarkan ambang batas)
- Proteksi sumber kosong (tidak mengisi jika sumber < threshold)
- Indikator LED peringatan jika drum C hampir kosong
- Tampilan LCD I2C (opsional, aktifkan dengan `GUNAKAN_LCD`)
- Logging data ke SD Card (opsional)
- Parameter kontrol dapat diubah via CLI USB VCP dan disimpan ke Flash

## Struktur File
- `Core/Src/main.c` : Logika utama, CLI, kontrol pengisian, komunikasi sensor
- `Core/Inc/main.h` : Header utama
- `Core/Src/lcd_i2c.c` & `Core/Inc/lcd_i2c.h` : Driver LCD I2C
- `FATFS/` : Library dan driver SD Card
- `USB_DEVICE/` : Library USB device (CDC/Virtual COM Port)

## Cara Kerja
1. Setiap sensor di-polling: dikirim 0xA0, menerima 3 byte data (H, M, L) + checksum (opsional).
2. Data sensor diproses state machine, validasi checksum jika ada.
3. Level drum dihitung dalam cm dan persen, berdasarkan parameter yang bisa diubah via CLI.
4. Logika kontrol:
   - Drum A diisi dari B jika level A < ambang bawah dan B > sumber kosong.
   - Drum B diisi dari C jika level B < ambang bawah dan C > sumber kosong, hanya jika A tidak sedang diisi.
   - Pengisian berhenti jika level target tercapai atau sumber kosong.
5. LED merah menyala jika drum C < ambang bawah.
6. LCD menampilkan status level dan pengisian.
7. Semua parameter (tinggi, ambang bawah, target penuh, sumber kosong) dapat diubah via CLI dan disimpan ke Flash.

## CLI USB VCP
- Hubungkan ke USB, buka serial terminal (baudrate bebas, 8N1)
- Perintah utama:
  - `$$` : Lihat semua parameter
  - `$P` : Lihat daftar pin
  - `$H` : Bantuan perintah
  - `$1=<min>,<max>` : Set ambang bawah/target penuh Drum A (misal: `$1=20,90`)
  - `$2=<min>,<max>` : Set Drum B
  - `$3=<min>,<max>` : Set Drum C
  - `$S` : Simpan parameter ke Flash
  - `$L` : Muat parameter dari Flash
  - `$D` : Kembalikan ke default

## Pinout & Hardware
- Sensor ultrasonik: USART1 (A), USART2 (B), USART3 (C)
- MAX485 DE: GPIOA/B (lihat kode, opsional untuk RS485)
- LCD I2C: I2C1 (PB6=SCL, PB7=SDA)
- SD Card: SPI1
- Kontrol valve/pompa: GPIOB (POMPA_BA_Pin, POMPA_CB_Pin)
- LED indikator: GPIOB (LED_RED_Pin, LED_GREEN_Pin)

## Kompilasi & Flash
1. Buka project di STM32CubeIDE.
2. Build project.
3. Flash ke board STM32F103CBT6.

## Catatan
- Fitur LCD dan SD Card dapat diaktifkan dengan mengubah macro `GUNAKAN_LCD` dan `GUNAKAN_SD_CARD` di `main.c`.
- Parameter kontrol dapat diubah via CLI USB VCP dan disimpan ke Flash internal.
- Pastikan wiring sensor dan aktuator sesuai pin yang digunakan.

## Lisensi
Copyright (c) 2025 STMicroelectronics.
