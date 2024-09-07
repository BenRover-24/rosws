import cv2

def read_ip_stream(ip_address):
    # Créer un objet VideoCapture avec l'adresse IP du flux
    cap = cv2.VideoCapture(ip_address)

    if not cap.isOpened():
        print("Erreur: Impossible d'ouvrir le flux vidéo")
        return

    while True:
        # Lire une image du flux
        ret, frame = cap.read()

        if not ret:
            print("Erreur: Impossible de lire une image du flux")
            break

        # Afficher l'image
        cv2.imshow('IP Camera Stream', frame)

        # Sortir de la boucle si 'q' est pressé
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Libérer les ressources
    cap.release()
    cv2.destroyAllWindows()

# Exemple d'utilisation
if __name__ == "__main__":
    # Remplacez cette URL par l'adresse de votre flux vidéo IP
    ip_stream_address = "http://10.61.16.209:4747/video"
    read_ip_stream(ip_stream_address)