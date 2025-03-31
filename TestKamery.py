import cv2
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Błąd: Nie można połączyć się z kamerą!")
else:
    print("Kamera działa poprawnie")
    ret, frame = cap.read()
    if ret:
        print("Obraz został poprawnie odczytany")
    else:
        print("Błąd odczytu obrazu")
    cap.release()