#Betounis Robotics
#kakoshund@yahoo.com

import cv2
import numpy as np
import tkinter as tk
from tkinter import filedialog, Label, Button
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import time
import math

USE_SERVOS = True  # False μόνο για test χωρίς τους αναλογικούς ηλεκτροκινητήρες
DEBUG_MODE = True  # Εμφάνιση debug μηνυμάτων

# Servo Pins
servo_pin_base = 17    # Βραχίονας αγκώνα
servo_pin_elbow = 27   # Βραχίονας βάσης

# Μήκη βραχιόνων (σε mm)
L1 = 200  # Μήκος πρώτου μέλους του βραχίονα
L2 = 200  # Μήκος δεύτερου δεύτερου μέλους του βραχίονα

# Αρχική θέση (θα οριστεί κατά τη χειροκίνητη ρύθμιση)
initial_position = {'x': 0, 'y': 0, 'base_angle': 90, 'elbow_angle': 90}

# Αρχικοποίηση Servo
if USE_SERVOS:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(servo_pin_base, GPIO.OUT)
    GPIO.setup(servo_pin_elbow, GPIO.OUT)
    servo_base = GPIO.PWM(servo_pin_base, 50)  # 50Hz PWM
    servo_elbow = GPIO.PWM(servo_pin_elbow, 50)
    servo_base.start(0)
    servo_elbow.start(0)

def set_servo_angle(servo, angle):
    """Βελτιωμένη συνάρτηση ελέγχου κινητήραςμοτέρ"""
    if not USE_SERVOS:
        if DEBUG_MODE:
            print(f"[SIM] {servo} → {angle}°")
        return
    
    angle = max(0, min(180, angle))
    duty = angle / 18 + 2
    
    try:
        if servo == "base":
            servo_base.ChangeDutyCycle(duty)
            if DEBUG_MODE:
                print(f"[BASE] {angle}° (Duty: {duty:.1f}%)")
        else:
            servo_elbow.ChangeDutyCycle(duty)
            if DEBUG_MODE:
                print(f"[ELBOW] {angle}° (Duty: {duty:.1f}%)")
        
        time.sleep(0.02)  # Μικρή καθυστέρηση για σταθερότητα
        
    except Exception as e:
        print(f"[ERROR] Σφάλμα κίνησης {servo}: {str(e)}")

# Αντίστροφη κινηματική για αρθρωτό βραχίονα
def inverse_kinematics(x, y):
    # Μετατόπιση συντεταγμένων ως προς τη θέση αρχής
    rel_x = x - initial_position['x']
    rel_y = y - initial_position['y']

    # Απόσταση στόχου από άρθρωση
    r = math.hypot(rel_x, rel_y)
    if r > (L1 + L2):
        print(f"Προειδοποίηση: Σημείο ({x},{y}) εκτός εμβέλειας!")
        return initial_position['base_angle'], initial_position['elbow_angle']

    try:
        # Υπολογισμός γωνίας αγκώνα με νόμο του συνημιτόνου
        cos_theta2 = (rel_x**2 + rel_y**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_theta2 = max(-1, min(1, cos_theta2))
        theta2 = math.acos(cos_theta2)

        # Επιλογή elbow-down λύσης (για τον ρομποτικό βραχίονα της εικόνας)
        sin_theta2 = math.sin(theta2)
        k1 = L1 + L2 * cos_theta2
        k2 = L2 * sin_theta2

        theta1 = math.atan2(rel_y, rel_x) - math.atan2(k2, k1)

        # Μετατροπή σε μοίρες
        theta1_deg = math.degrees(theta1)
        theta2_deg = math.degrees(theta2)

        # Αντιστροφή αν ο προσανατολισμός κινητήρας είναι ανάποδος (αν κινείται ανάποδα)
        theta1_deg = 180 - theta1_deg  # Αν χρειάζεται προσαρμογή μηχανική
        theta2_deg = theta2_deg  # Διατήρηση της γωνίας αγκώνα

        # Ορισμός τελικών γωνιών σε εύρος 0–180
        theta1_deg = max(0, min(180, theta1_deg))
        theta2_deg = max(0, min(180, theta2_deg))

        return theta1_deg, theta2_deg

    except Exception as e:
        print(f"[Σφάλμα IK] {e}")
        return initial_position['base_angle'], initial_position['elbow_angle']

# Επιλογή εικόνας και χαρτιού (χωρίς αλλαγές)
def select_image_and_paper():
    selected = {"file": None, "paper": None}

    def choose_file():
        path = filedialog.askopenfilename(
            initialdir="/media/pi",
            title="Επιλέξτε εικόνα",
            filetypes=[("Image files", "*.png;*.jpg;*.jpeg;*.tif")]
        )
        if path:
            selected["file"] = path

    def choose_paper(size):
        selected["paper"] = size
        root.quit()

    root = tk.Tk()
    root.title("Επιλογή Εικόνας και Χαρτιού")

    Button(root, text="Επιλέξτε Εικόνα", command=choose_file).pack(pady=10)
    Label(root, text="Επιλέξτε μέγεθος χαρτιού", font=("Arial", 12)).pack(pady=10)
    Button(root, text="A4 (210x297mm)", width=20, command=lambda: choose_paper("A4")).pack(pady=5)
    Button(root, text="A3 (297x420mm)", width=20, command=lambda: choose_paper("A3")).pack(pady=5)

    root.mainloop()
    root.destroy()

    return selected["file"], selected["paper"]

def manual_initialize_position():
    """Τελική έκδοση χειροκίνητης αρχικοποίησης"""
    global initial_position
    
    # Αρχικές γωνίες
    base_angle = 90
    elbow_angle = 90
    step_size = 3  # Μοίρες μετακίνησης ανά κλικ στα κουμπιά
    
    # Αρχική θέση κινητήρα
    set_servo_angle("base", base_angle)
    set_servo_angle("elbow", elbow_angle)
    time.sleep(0.5)

    def update_position(delta_base=0, delta_elbow=0):
        """Ενημέρωση θέσης με έλεγχο ορίων"""
        nonlocal base_angle, elbow_angle
        
        base_angle = max(0, min(180, base_angle + delta_base))
        elbow_angle = max(0, min(180, elbow_angle + delta_elbow))
        
        set_servo_angle("base", base_angle)
        set_servo_angle("elbow", elbow_angle)
        
        if DEBUG_MODE:
            print(f"Θέση: Βάση={base_angle}°, Αγκώνας={elbow_angle}°")

    def move_left():
        update_position(delta_base=-step_size)
        
    def move_right():
        update_position(delta_base=step_size)
        
    def move_up():
        update_position(delta_elbow=step_size)
        
    def move_down():
        update_position(delta_elbow=-step_size)

    def save_position():
        """Υπολογισμός θέσης γραφίδας και αποθήκευση"""
        nonlocal base_angle, elbow_angle
        
        # Υπολογισμός συντεταγμένων (x,y) από τις τρέχουσες γωνίες
        theta1 = math.radians(base_angle)
        theta2 = math.radians(elbow_angle)
        
        # Υπολογισμός θέσης άκρου βραχίονα (forward kinematics)
        x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
        y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
        
        # Αποθήκευση ως αρχική θέση (0,0)
        initial_position.update({
            'x': 0,  # Θέτουμε το τρέχον σημείο ως (0,0)
            'y': 0,
            'base_angle': base_angle,
            'elbow_angle': elbow_angle
        })
        
        print(f"\nΘΕΣΗ ΜΗΔΕΝΙΣΜΟΥ ΟΡΙΣΤΗΚΕ:")
        print(f"- Συντεταγμένες: (0, 0) mm (υπολογισμένες από γωνίες)")
        print(f"- Γωνίες: Βάση={base_angle}°, Αγκώνας={elbow_angle}°")
        
        # Κλείσιμο παραθύρου
        root.destroy()

    # Δημιουργία GUI
    root = tk.Tk()
    root.title("Αρχικοποίηση Θέσης Μηδενισμού")
    root.geometry("500x350")
    
    # Στυλ κουμπιών
    btn_style = {
        'font': ('Arial', 12),
        'width': 10,
        'height': 2
    }
    
    Label(root, text="Τοποθετήστε τη γραφίδα στην επάνω αριστερή γωνία του χαρτιού", 
          font=('Arial', 14)).pack(pady=10)
    
    # Πλαίσιο ελέγχου
    control_frame = tk.Frame(root)
    control_frame.pack(pady=15)
    
    # Σέρβο 1 (Βάση - Αριστερά/Δεξιά)
    servo1_frame = tk.Frame(control_frame)
    servo1_frame.grid(row=0, column=0, padx=10)
    
    Label(servo1_frame, text="Κινητήρας Άρθρωσης", font=('Arial', 12, 'bold')).pack()
    Button(servo1_frame, text="Αριστερά", command=move_left, **btn_style).pack(pady=5)
    Button(servo1_frame, text="Δεξιά", command=move_right, **btn_style).pack(pady=5)
    
    # Σέρβο 2 (Αγκώνας - Πάνω/Κάτω που τώρα είναι Αριστερά/Δεξιά)
    servo2_frame = tk.Frame(control_frame)
    servo2_frame.grid(row=0, column=1, padx=10)
    
    Label(servo2_frame, text="Κινητήρας Βάσης", font=('Arial', 12, 'bold')).pack()
    Button(servo2_frame, text="Αριστερά", command=move_up, **btn_style).pack(pady=5)
    Button(servo2_frame, text="Δεξιά", command=move_down, **btn_style).pack(pady=5)
    
    # Κουμπί ορισμού θέσης
    Button(root, text="ΟΡΙΣΜΟΣ ΑΡΧΙΚΗΣ ΘΕΣΗΣ (0,0)", 
           command=save_position, 
           font=('Arial', 12, 'bold'),
           bg='green', fg='white',
           height=2, width=25).pack(pady=20)
    
    # Εναλλακτικός έλεγχος με πλήκτρα
    root.bind('<Left>', lambda e: move_left())
    root.bind('<Right>', lambda e: move_right())
    root.bind('<Up>', lambda e: move_up())
    root.bind('<Down>', lambda e: move_down())
    root.focus_set()
    
    root.mainloop()

# Βελτιωμένη επεξεργασία εικόνας (χωρίς αλλαγές)
def process_image(image_path):
    original_color = cv2.imread(image_path, cv2.IMREAD_COLOR)  # Διατήρηση έγχρωμης για εμφάνιση στην συνέχεια 
    image_gray = cv2.cvtColor(original_color, cv2.COLOR_BGR2GRAY)  # Μετατροπή σε γκρι για την ανίχνευση ακμών
    image_gray = cv2.equalizeHist(image_gray)

    sobelx = cv2.Sobel(image_gray, cv2.CV_64F, 1, 0, ksize=5)
    sobely = cv2.Sobel(image_gray, cv2.CV_64F, 0, 1, ksize=5)
    sobel_combined = cv2.magnitude(sobelx, sobely)
    sobel_combined = np.uint8(255 * sobel_combined / np.max(sobel_combined))

    _, thresh = cv2.threshold(sobel_combined, 50, 255, cv2.THRESH_BINARY)
    inverted_edges = cv2.bitwise_not(thresh)
    kernel = np.ones((3, 3), np.uint8)
    inverted_edges = cv2.morphologyEx(inverted_edges, cv2.MORPH_CLOSE, kernel)

    return original_color, sobel_combined, inverted_edges

# Κλίμακα περιγραμμάτων (τροποποιημένη για να λαμβάνει υπόψη την αρχική θέση)
def scale_contours_to_paper(image_edges, paper_size):
    img_height, img_width = image_edges.shape
    
    if paper_size == "A3":
        paper_w, paper_h = 297, 420  # Σε mm
    elif paper_size == "A4":
        paper_w, paper_h = 210, 297  # Σε mm
    else:
        raise ValueError("Άγνωστο μέγεθος χαρτιού")

    scale_x = paper_w / img_width
    scale_y = paper_h / img_height

    contours, _ = cv2.findContours(image_edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    scaled_contours = []
    for contour in contours:
        scaled_contour = []
        for point in contour:
            x, y = point[0]
            # Κλιμάκωση - η αρχική θέση είναι πλέον πάντα (0,0)
            scaled_x = x * scale_x
            scaled_y = y * scale_y
            scaled_contour.append((scaled_x, scaled_y))
        scaled_contours.append(scaled_contour)

    return scaled_contours, paper_w, paper_h

# Σχεδίαση με αρθρωτό βραχίονα (τροποποιημένη)
def draw_edges_with_arm(contours, ax, paper_w, paper_h):
    ax.clear()
    ax.set_facecolor('white')
    ax.set_xlim(initial_position['x'], initial_position['x'] + paper_w)
    ax.set_ylim(initial_position['y'] + paper_h, initial_position['y'])
    ax.set_aspect('equal')
    ax.set_title('Πρόοδος Σχεδίασης')
    plt.draw()
    
    if not contours:
        print("Προειδοποίηση: Δεν βρέθηκαν περιγράμματα για σχεδίαση!")
        return
    
    for i, contour in enumerate(contours):
        if len(contour) < 2:
            continue
        
        contour = np.array(contour).reshape(-1, 2)
        x, y = contour[:,0], contour[:,1]
        
        ax.plot(x, y, color="black", linewidth=1.5)
        plt.pause(0.05)
        
        for point_x, point_y in zip(x, y):
            theta1, theta2 = inverse_kinematics(point_x, point_y)
            set_servo_angle("base", theta1)
            set_servo_angle("elbow", theta2)
            time.sleep(0.01)

# ΚΥΡΙΟ ΠΡΟΓΡΑΜΜΑ
try:
    image_path, paper_size = select_image_and_paper()
    
    if not image_path or not paper_size:
        print("Ακύρωση: Δεν επιλέχθηκε εικόνα ή μέγεθος χαρτιού")
        exit()

    manual_initialize_position()

    original_image, sobel_image, inverted_edges = process_image(image_path)
    
    cv2.imshow('Ανεστραμμένες Ακμές', inverted_edges)
    cv2.waitKey(1000)
    cv2.destroyAllWindows()

    contours, paper_w, paper_h = scale_contours_to_paper(inverted_edges, paper_size)
    
    if not contours:
        print("Σφάλμα: Δεν βρέθηκαν περιγράμματα στην επεξεργασμένη εικόνα!")
        exit()

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))

    ax1.imshow(original_image, cmap='gray')
    ax1.set_title('Αρχική Εικόνα')
    ax1.axis('off')

    ax2.imshow(inverted_edges, cmap='gray')
    ax2.set_title('Ανεστραμμένο Sobel')
    ax2.axis('off')

    ax3.plot([initial_position['x'], initial_position['x'] + paper_w, 
              initial_position['x'] + paper_w, initial_position['x'], 
              initial_position['x']],
             [initial_position['y'], initial_position['y'], 
              initial_position['y'] + paper_h, initial_position['y'] + paper_h, 
              initial_position['y']], 'k--')
    ax3.set_xlim(initial_position['x'], initial_position['x'] + paper_w)
    ax3.set_ylim(initial_position['y'] + paper_h, initial_position['y'])
    ax3.set_title('Πρόοδος Σχεδίασης')
    ax3.set_aspect('equal')
    ax3.axis('on')

    print("Έναρξη σχεδίασης...")
    draw_edges_with_arm(contours, ax3, paper_w, paper_h)
    print("Ολοκλήρωση σχεδίασης!")

    plt.tight_layout()
    plt.show()

finally:
    if USE_SERVOS:
        servo_base.stop()
        servo_elbow.stop()
        GPIO.cleanup()
