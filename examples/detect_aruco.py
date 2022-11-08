import cv2
from pioneer_sdk import Camera

# Словарь aruco-маркеров
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
# Параметры детектирования (в данном случае параметры по умолчанию)
arucoParams = cv2.aruco.DetectorParameters_create()

# Камера дрона
camera = Camera()

while True:
    frame = camera.get_cv_frame()  # Получаем изображение
    # Детектируем маркеры
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
    # Обводим распознанные маркеры на изображении
    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    cv2.imshow('video', frame)  # Выводим изображение на экран
    
    if cv2.waitKey(1) & 0xFF == 27:  # Выход из программы, если нажали ESC
        break

cv2.destroyAllWindows()  # Закрываем все открытые окна
