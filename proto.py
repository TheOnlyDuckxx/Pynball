import math
import pygame

# --- Paramètres "jeu/physique" (faciles à régler) ---
WIDTH, HEIGHT = 600, 1024         # fenêtre en mode portrait (écran 10")
RADIUS = 16                        # rayon de la bille (px)
GRAVITY = 1000.0                   # intensité de la gravité (px/s^2)
SLOPE_DEG = 0.0                    # pente (degrés) : 0 = vertical ; >0 = penche vers la droite
REST_E = 0.78                      # coefficient de restitution (rebond)
AIR_DRAG = 0.998                   # frottement "air" par tick (1.0 = aucun)
GROUND_FRICTION = 0.96             # perte d'énergie horizontale au contact du sol
SUBSTEPS = 6                       # sous-étapes de physique par frame (évite le "tunneling")

# Paramètres des flippers
FLIPPER_LENGTH = 100
FLIPPER_WIDTH  = 20
FLIPPER_REST_ANGLE   = 45    # angle de repos (degrés, sens trigonométrique)
FLIPPER_ACTIVE_ANGLE = -20   # angle actif (vers le haut)
FLIPPER_SPEED = 18           # vitesse de rotation (degrés par frame) – utilisé pour easing
FLIPPER_IMPULSE_SCALE = 0.8  # modulateur global d'impulsion

# Paramètres du cadre de jeu
BORDER_WIDTH = 30  # Largeur des bords du cadre
GAP_WIDTH = 150    # Largeur du trou central entre les flippers

# Position et taille du trou entre les flippers
gap_x1 = WIDTH // 2 - GAP_WIDTH // 2
gap_x2 = WIDTH // 2 + GAP_WIDTH // 2
gap_y  = HEIGHT - 10  # Position verticale du trou

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Prototype Physique Pinball")

clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 20)

# --- État de la bille ---
x, y = WIDTH // 2, RADIUS + 10     # position initiale (en px)
vx, vy = 0.0, 0.0                  # vitesse initiale (px/s)

# --- Flipper ---
class Flipper:
    def __init__(self, x, y, side='left'):
        self.x = float(x)
        self.y = float(y)
        self.side = side

        if side == 'right':
            rest_deg   = 180 - FLIPPER_REST_ANGLE
            active_deg = 180 - FLIPPER_ACTIVE_ANGLE
        else:
            rest_deg   = FLIPPER_REST_ANGLE
            active_deg = FLIPPER_ACTIVE_ANGLE

        self.rest_angle   = math.radians(rest_deg)
        self.active_angle = math.radians(active_deg)

        self.angle        = self.rest_angle
        self.target_angle = self.rest_angle
        self.prev_angle   = self.angle    # <-- nécessaire pour la vitesse angulaire
        self.length = float(FLIPPER_LENGTH)
        self.width  = float(FLIPPER_WIDTH)
        self.active = False

    def update(self):
        # mémorise l'angle précédent pour calculer ω ensuite
        self.prev_angle = self.angle
        # poursuite vers la cible avec vitesse bornée (plus stable que un simple *0.3)
        diff = self.target_angle - self.angle
        max_step = math.radians(FLIPPER_SPEED)  # degrés/frame -> rad/frame
        step = max(-max_step, min(max_step, diff))
        self.angle += step

    def activate(self):
        self.target_angle = self.active_angle
        self.active = True

    def deactivate(self):
        self.target_angle = self.rest_angle
        self.active = False

    def get_end_point(self):
        end_x = self.x + math.cos(self.angle) * self.length
        end_y = self.y + math.sin(self.angle) * self.length
        return end_x, end_y

    def draw(self, surface):
        end_x, end_y = self.get_end_point()
        pygame.draw.line(surface, (255, 255, 255), (self.x, self.y), (end_x, end_y), int(self.width))
        pygame.draw.circle(surface, (200, 200, 200), (int(self.x), int(self.y)), int(self.width//2))

    def point_velocity(self, px, py, dt):
        """
        Vitesse du point (px,py) dûe à la rotation du flipper : v = ω × r.
        ω ≈ (angle - prev_angle) / dt (rad/s), r = (px - pivot, py - pivot)
        """
        if dt <= 0:
            return 0.0, 0.0
        w = (self.angle - self.prev_angle) / dt  # rad/s
        rx, ry = px - self.x, py - self.y
        # En 2D : ω × r = (-ω*ry, ω*rx)
        return (-w * ry, w * rx)


# Créarrtion des flippers
flipper_left  = Flipper(WIDTH//3,     HEIGHT - 50, 'left')
flipper_right = Flipper(2*WIDTH//3,   HEIGHT - 50, 'right')


def gravity_vector(angle_deg: float, g: float):
    """Gravité orientée selon la pente.
    angle en degrés mesuré depuis la verticale vers la droite.
    0° -> vertical; 10° -> bas + un peu droite.
    """
    a = math.radians(angle_deg)
    gx = g * math.sin(a)
    gy = g * math.cos(a)
    return gx, gy


def check_wall_and_hole(ball_x, ball_y):
    """Retourne (fell_in_hole, new_x, new_y, new_vx, new_vy)."""
    global vx, vy
    # trou
    if ball_y > gap_y and gap_x1 < ball_x < gap_x2:
        return True, ball_x, ball_y, vx, vy

    # murs latéraux
    if ball_x < RADIUS + BORDER_WIDTH:
        ball_x = RADIUS + BORDER_WIDTH
        vx = -vx * REST_E
    elif ball_x > WIDTH - RADIUS - BORDER_WIDTH :
        ball_x = WIDTH - RADIUS - BORDER_WIDTH
        vx = -vx * REST_E

    # plafond
    if ball_y < RADIUS + BORDER_WIDTH:
        ball_y = RADIUS + BORDER_WIDTH
        vy = -vy * REST_E

    # sol (hors trou)
    if ball_y > HEIGHT - RADIUS - BORDER_WIDTH -20 and (ball_x <= gap_x1 or ball_x >= gap_x2):
        ball_y = HEIGHT - RADIUS - BORDER_WIDTH -20
        vy = -vy * REST_E
        vx *= GROUND_FRICTION

    return False, ball_x, ball_y, vx, vy


def resolve_flipper_collision(ball_x, ball_y, ball_vx, ball_vy, flipper, dt):
    """Collision capsule (segment+rayon) / disque, avec réflexion du vecteur vitesse.
    Retourne (x,y,vx,vy,hit)
    """
    fx, fy = flipper.x, flipper.y
    ex, ey = flipper.get_end_point()
    dx, dy = ex - fx, ey - fy
    seg_len = math.hypot(dx, dy)
    if seg_len <= 1e-6:
        return ball_x, ball_y, ball_vx, ball_vy, False

    ux, uy = dx/seg_len, dy/seg_len  # direction du segment

    # projection du centre de la bille sur le segment
    bx, by = ball_x - fx, ball_y - fy
    t = bx*ux + by*uy
    t_clamped = max(0.0, min(seg_len, t))
    closest_x = fx + ux * t_clamped
    closest_y = fy + uy * t_clamped

    # test pénétration avec rayon élargi (capsule)
    rx = ball_x - closest_x
    ry = ball_y - closest_y
    dist = math.hypot(rx, ry)
    min_dist = RADIUS + flipper.width*0.5

    if dist >= min_dist:
        return ball_x, ball_y, ball_vx, ball_vy, False

    # normale sortante (du flipper vers la bille); si dist quasi nul, prend une normale au segment
    if dist > 1e-6:
        nx, ny = rx/dist, ry/dist
    else:
        nx, ny = -uy, ux  # perpendiculaire arbitraire

    # pousse la bille hors de la capsule (position corrective)
    penetration = (min_dist - dist)
    ball_x += nx * penetration
    ball_y += ny * penetration

    # vitesse du point de contact lié au flipper (flipper "actif" bouge)
    contact_x = closest_x
    contact_y = closest_y
    pvx, pvy = flipper.point_velocity(contact_x, contact_y, dt)

    # vitesse relative le long de la normale
    rel_vx = ball_vx - pvx
    rel_vy = ball_vy - pvy
    rel_n  = rel_vx*nx + rel_vy*ny

    # Si on s'éloigne déjà (rel_n>0), ne pas re‑réfléchir; sinon, réflexion + restitution
    if rel_n < 0.0:
        j = -(1.0 + REST_E) * rel_n  # quantité de changement sur la composante normale
        ball_vx += j * nx
        ball_vy += j * ny

        # petit boost si flipper actif (impulsion additionnelle contrôlée)
        if flipper.active:
            ball_vx += nx * 120 * FLIPPER_IMPULSE_SCALE
            ball_vy += ny *  90 * FLIPPER_IMPULSE_SCALE

    return ball_x, ball_y, ball_vx, ball_vy, True


def step_physics(dt: float):
    global x, y, vx, vy, running


    fell, _x, _y, _vx, _vy = check_wall_and_hole(x, y)
    if fell:
        x, y = WIDTH // 2, RADIUS + 10
        vx, vy = 0.0, 0.0

    # 1) gravité
    gx, gy = gravity_vector(SLOPE_DEG, GRAVITY)
    vx += gx * dt
    vy += gy * dt

    # 2) intégration
    x_try = x + vx * dt
    y_try = y + vy * dt

    # 3) collisions murs & trou
    fell, x_try, y_try, vx, vy = check_wall_and_hole(x_try, y_try)

    # 4) collisions flippers (ordre: gauche puis droit)
    x_try, y_try, vx, vy, _ = resolve_flipper_collision(x_try, y_try, vx, vy, flipper_left,  dt)
    x_try, y_try, vx, vy, _ = resolve_flipper_collision(x_try, y_try, vx, vy, flipper_right, dt)

    # 5) affectation
    x, y = x_try, y_try

    # 6) frottements d'air
    vx *= AIR_DRAG
    vy *= AIR_DRAG


running = True
while running:
    # --- Gestion des entrées clavier ---
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        flipper_left.activate()
    else:
        flipper_left.deactivate()

    if keys[pygame.K_RIGHT]:
        flipper_right.activate()
    else:
        flipper_right.deactivate()

    # --- Boucle événements ---
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r:  # reset
                x, y = WIDTH // 2, RADIUS + 10
                vx, vy = 0.0, 0.0

    # Mise à jour des flippers (conserve prev_angle pour vitesse angulaire)
    flipper_left.update()
    flipper_right.update()

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

    # cadre de jeu
    border_color = (100, 100, 200)
    # haut
    pygame.draw.rect(screen, border_color, (0, 0, WIDTH, BORDER_WIDTH))
    # côtés
    pygame.draw.rect(screen, border_color, (0, 0, BORDER_WIDTH, HEIGHT))
    pygame.draw.rect(screen, border_color, (WIDTH - BORDER_WIDTH, 0, BORDER_WIDTH, HEIGHT))
    # bas avec trou
    pygame.draw.rect(screen, border_color, (0, HEIGHT - BORDER_WIDTH - 20, gap_x1, BORDER_WIDTH))
    pygame.draw.rect(screen, border_color, (gap_x2, HEIGHT - BORDER_WIDTH - 20, WIDTH - gap_x2, BORDER_WIDTH))
    pygame.draw.rect(screen, (0, 0, 0), (gap_x1, gap_y - 10, GAP_WIDTH, BORDER_WIDTH + 10))

    # flippers
    flipper_left.draw(screen)
    flipper_right.draw(screen)

    # bille
    pygame.draw.circle(screen, (200, 50, 50), (int(x), int(y)), RADIUS)

    # debug
    debug_text = [
        f"Position: ({x:.1f}, {y:.1f})",
        f"Vitesse: ({vx:.1f}, {vy:.1f})",
        f"Pente: {SLOPE_DEG}°",
        "Contrôles:",
        "- ← : flipper gauche",
        "- → : flipper droit",
        "- R : reset",
    ]
    for i, text in enumerate(debug_text):
        surf = font.render(text, True, (200, 200, 200))
        screen.blit(surf, (10, 10 + i * 20))

    pygame.display.flip()

pygame.quit()
