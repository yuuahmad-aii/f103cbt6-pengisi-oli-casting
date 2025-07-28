# STM32F103CBT6 Pengisi Oli Casting

## Deskripsi
Proyek ini adalah sistem otomatis pengisian oli menggunakan mikrokontroler STM32F103CBT6. Sistem mengontrol pengisian tiga drum (A, B, C) berdasarkan level sensor ultrasonik, dengan prioritas pengisian, histeresis, dan pengaman sumber kosong. Terdapat fitur tampilan LCD I2C dan logging ke SD Card (opsional).

## Fitur Utama
- Kontrol otomatis pengisian drum dengan prioritas (A > B > C)
- Sensor level menggunakan UART (ultrasonik)
- Histeresis pengisian (start/stop berdasarkan ambang batas)
- Proteksi sumber kosong (tidak mengisi jika sumber < 1%)
- Indikator LED peringatan jika drum C hampir kosong
- Tampilan LCD I2C (opsional)
- Logging data ke SD Card (opsional)

## Struktur File
- `Core/Src/main.c` : Logika utama, inisialisasi, kontrol pengisian, dan komunikasi sensor
- `Core/Inc/main.h` : Header utama
- `Core/Src/lcd_i2c.c` & `Core/Inc/lcd_i2c.h` : Driver LCD I2C
- `FATFS/` : Library dan driver SD Card
- `USB_DEVICE/` : Library USB device (opsional)

## Cara Kerja
1. Setiap sensor mengirim data jarak via UART (interrupt).
2. Data sensor diproses dengan state machine dan validasi checksum.
3. Level drum dihitung dalam cm dan persen.
4. Logika kontrol:
   - Drum A diisi dari B jika level A < 25% dan B > 1%.
   - Drum B diisi dari C jika level B < 25% dan C > 1%, hanya jika A tidak sedang diisi.
   - Pengisian berhenti jika level target tercapai atau sumber kosong.
5. LED merah menyala jika drum C < 25%.
6. LCD menampilkan status level dan pengisian.

## Pinout & Hardware
- Sensor ultrasonik: UART1, UART2, UART3
- LCD I2C: I2C1
- SD Card: SPI1
- Kontrol valve/pompa: GPIOB (POMPA_BA_Pin, POMPA_CB_Pin)
- LED indikator: GPIOB (LED_RED_Pin, LED_GREEN_Pin)

## Kompilasi & Flash
1. Buka project di STM32CubeIDE.
2. Build project.
3. Flash ke board STM32F103CBT6.

## Catatan
- Fitur LCD dan SD Card dapat diaktifkan dengan mengubah macro `GUNAKAN_LCD` dan `GUNAKAN_SD_CARD` di `main.c`.
- Pastikan wiring sensor dan aktuator sesuai pin yang digunakan.

## Lisensi
Copyright (c) 2025 STMicroelectronics.
