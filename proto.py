import math
import pygame

# --- Paramètres "jeu/physique" (faciles à régler) ---
WIDTH, HEIGHT = 600, 1024         # fenêtre en mode portrait (écran 10")
RADIUS = 16                        # rayon de la bille (px)
GRAVITY = 1000.0                   # intensité de la gravité (px/s^2)
SLOPE_DEG = 0.0                    # pente (degrés) : 0 = vertical ; >0 = penche vers la droite
REST_E = 0.75                      # coefficient de restitution (rebond) : 1.0 = rebond parfait ; 0.7–0.85 "réaliste"
AIR_DRAG = 0.999                   # frottement "air" par tick (1.0 = aucun)
GROUND_FRICTION = 0.96             # perte d'énergie horizontale au contact du sol
SUBSTEPS = 3                       # sous-étapes de physique par frame (évite de "traverser" le sol)

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Prototype Physique Pinball")

clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 20)

# --- État de la bille ---
x, y = WIDTH // 2, RADIUS + 10     # position initiale (en px)
vx, vy = 0.0, 0.0                  # vitesse initiale (px/s)

def gravity_vector(angle_deg: float, g: float):
    """
    Gravité orientée selon la pente.
    Convention : angle mesuré depuis la verticale vers la droite.
      - 0°  -> gravité purement vers le bas
      - 10° -> gravité "penchée" vers le bas + un peu vers la droite
    """
    a = math.radians(angle_deg)
    gx = g * math.sin(a)   # composante horizontale
    gy = g * math.cos(a)   # composante verticale
    return gx, gy

def step_physics(dt: float):
    global x, y, vx, vy

    # 1) Accélération due à la gravité (orientée par la pente)
    gx, gy = gravity_vector(SLOPE_DEG, GRAVITY)
    vx += gx * dt
    vy += gy * dt

    # 2) Intégration position
    x += vx * dt
    y += vy * dt

    # 3) Collisions murs gauche/droit (pour garder la bille dans le cadre)
    if x < RADIUS:
        x = RADIUS
        vx = -vx * REST_E
    elif x > WIDTH - RADIUS:
        x = WIDTH - RADIUS
        vx = -vx * REST_E

    # 4) Collision sol (bas de l'écran)
    floor = HEIGHT - RADIUS
    if y > floor:
        y = floor
        vy = -vy * REST_E        # inversion + perte d'énergie
        vx *= GROUND_FRICTION    # frottement au contact
        # Option : "endormir" la bille si elle ne bouge presque plus
        if abs(vy) < 20:
            vy = 0

    # 5) Frottements "air"
    vx *= AIR_DRAG
    vy *= AIR_DRAG

running = True
while running:
    # --- Boucle événements ---
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # Raccourcis utiles pendant vos tests
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r:  # reset
                x, y = WIDTH // 2, RADIUS + 10
                vx, vy = 0.0, 0.0
            elif event.key == pygame.K_LEFT:
                SLOPE_DEG -= 1
            elif event.key == pygame.K_RIGHT:
                SLOPE_DEG += 1

    # --- Physique (avec sous-étapes pour la stabilité) ---
    dt = clock.tick(60) / 1000.0
    if SUBSTEPS > 1:
        sub_dt = dt / SUBSTEPS
        for _ in range(SUBSTEPS):
            step_physics(sub_dt)
    else:
        step_physics(dt)

    # --- Rendu ---
    screen.fill((12, 12, 14))
    # Ligne du "sol" pour repère visuel
    pygame.draw.line(screen, (60, 60, 70), (0, HEIGHT - RADIUS), (WIDTH, HEIGHT - RADIUS), 2)
    # La bille
    pygame.draw.circle(screen, (220, 60, 60), (int(x), int(y)), RADIUS)

    # Overlay d'infos (utile pendant le tuning)
    info = f"Slope={SLOPE_DEG:.1f}°, v=({vx:.1f},{vy:.1f}) px/s, dt={dt*1000:.2f} ms"
    screen.blit(font.render(info, True, (200, 200, 210)), (10, 10))

    pygame.display.flip()

pygame.quit()