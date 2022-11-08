import cv2
import numpy as np
from pioneer_sdk import Camera

# Камера дрона
camera = Camera()

while True:
    raw_frame = camera.get_frame()  # Получаем сырые данные
    if raw_frame is not None:
        # Декодируем полученные данные, чтобы получить изображение
        frame = camera_frame = cv2.imdecode(np.frombuffer(raw_frame, dtype=np.uint8), cv2.IMREAD_COLOR)
    cv2.imshow('video', frame)  # Выводим изображение на экран
    
    if cv2.waitKey(1) & 0xFF == 27:  # Выход из программы, если нажали ESC
        break

cv2.destroyAllWindows()  # Закрываем все открытые окна