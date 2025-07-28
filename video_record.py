import cv2
import datetime

# Параметры
camera_index = 0  # 0 — обычно первая USB-камера
frame_width = 640
frame_height = 480
fps = 20.0
output_filename = f"video_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.avi"

# Захват видео с камеры
cap = cv2.VideoCapture(camera_index)

# Настройка разрешения
cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

# Проверка открытия камеры
if not cap.isOpened():
    print("Ошибка: не удалось открыть камеру")
    exit()

# Кодек и выходной файл
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(output_filename, fourcc, fps, (frame_width, frame_height))

print(f"Запись начата. Нажмите 'q' для остановки. Файл будет сохранён как {output_filename}")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Ошибка при чтении кадра")
        break

    out.write(frame)
    
    # Отображение (можно убрать, если не нужно окно)
    cv2.imshow('Запись', frame)

    # Нажмите 'q' для выхода
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Остановка записи")
        break

# Освобождение ресурсов
cap.release()
out.release()
cv2.destroyAllWindows()



### Unit script
[Unit]
Description=Автоматическая запись видео с USB-камеры
After=multi-user.target

[Service]
Type=simple
ExecStart=/usr/bin/python3 /home/username/record_video.py
WorkingDirectory=/home/username
StandardOutput=journal
StandardError=journal
Restart=on-failure

[Install]
WantedBy=multi-user.target
