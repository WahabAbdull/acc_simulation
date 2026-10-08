import sys
import pygame
import numpy as np

# -----------------------------
# Display & Simulation Constants
# -----------------------------
SCREEN_WIDTH = 1080
SCREEN_HEIGHT = 720
PPM = 8.0  # Scale: 8 pixels = 1 meter
ROAD_Y = 360
DT = 0.05  # Time step in seconds (20 Hz update)

# Visual Colors
COLOR_BG = (240, 243, 246)
COLOR_ROAD = (50, 55, 65)
COLOR_MARKING = (255, 255, 255)
COLOR_EGO = (30, 110, 230)       # Blue: Ego Vehicle
COLOR_LEAD = (230, 80, 50)       # Red/Orange: Leading Vehicle
COLOR_RADAR = (40, 167, 69, 60)  # Green translucent cone
COLOR_TEXT = (30, 30, 30)


class Vehicle:
    def __init__(self, x, v, length=4.5, width=2.0):
        self.x = float(x)        # Position along road (meters)
        self.v = float(v)        # Velocity (m/s)
        self.a = 0.0             # Current acceleration (m/s^2)
        self.length = length     # Physical length (meters)
        self.width = width

    def update(self, accel_cmd, dt):
        # Clip commanded acceleration to physical limits
        self.a = float(np.clip(accel_cmd, -6.0, 2.5))  # Max braking: -6 m/s^2, Max gas: 2.5 m/s^2
        self.v = max(0.0, self.v + self.a * dt)
        self.x += self.v * dt


class AdaptiveCruiseController:
    """
    Longitudinal Controller incorporating:
    1. Cruise Control (Velocity Tracking)
    2. Adaptive Following (Distance Keeping)
    3. Emergency Braking via Time-to-Collision (TTC)
    """
    def __init__(self, target_v=22.0, time_gap=1.5, d_standstill=6.0):
        self.target_v = target_v          # Desired cruising speed (~80 km/h)
        self.time_gap = time_gap          # Desired time gap h (seconds)
        self.d_standstill = d_standstill  # Safe standstill gap s0 (meters)
        self.kp_v = 0.8                   # Speed error gain
        self.kp_d = 1.2                   # Distance error gain
        self.kv_rel = 1.5                 # Relative velocity damping gain
        self.state = "CRUISE"

    def compute_control(self, ego, lead_detected, lead_dist, v_lead):
        # 1. No lead vehicle in sensor range -> Standard Speed Control
        if not lead_detected:
            self.state = "CRUISE"
            accel = self.kp_v * (self.target_v - ego.v)
            return accel, None

        # Relative speed (positive if ego is closing in on lead)
        v_rel = ego.v - v_lead

        # 2. Time-to-Collision (TTC) Critical Safety Layer
        if v_rel > 0.1:
            ttc = lead_dist / v_rel
        else:
            ttc = float('inf')

        # Emergency override if collision imminent within 1.8 seconds
        if ttc < 1.8 and lead_dist < 25.0:
            self.state = "EMERGENCY BRAKE"
            return -6.0, ttc  # Maximum deceleration

        # 3. Safe Following Distance Control (Spacing Policy)
        # Desired Gap: s_star = d_standstill + ego.v * time_gap
        desired_gap = self.d_standstill + (ego.v * self.time_gap)
        gap_error = lead_dist - desired_gap

        # Control law: combine distance gap error and relative velocity
        self.state = "FOLLOWING"
        accel = (self.kp_d * gap_error) - (self.kv_rel * v_rel)
        return accel, ttc


def to_screen_x(x_meters, camera_offset_x):
    return int((x_meters - camera_offset_x) * PPM + 120)


def main():
    pygame.init()
    pygame.font.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Autonomous Adaptive Cruise Control (ACC) & AEB Simulation")
    clock = pygame.time.Clock()

    # Safe font initialization
    try:
        font = pygame.font.SysFont("Arial", 16)
        bold_font = pygame.font.SysFont("Arial", 18, bold=True)
    except Exception:
        font = pygame.font.Font(None, 20)
        bold_font = pygame.font.Font(None, 24)

    ego_car = Vehicle(x=0.0, v=15.0)
    lead_car = Vehicle(x=45.0, v=15.0)  # Starts 45m ahead
    acc = AdaptiveCruiseController(target_v=22.0, time_gap=1.5, d_standstill=6.0)

    sim_time = 0.0
    radar_range = 60.0  # Max sensor detection envelope (meters)
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False

        # --- A. Dynamic Lead Vehicle Behavior (Test Scenario) ---
        sim_time += DT
        if sim_time < 5.0:
            lead_accel = 0.0                   # Cruise at initial speed
        elif sim_time < 11.0:
            lead_accel = -2.2                  # Moderate deceleration
        elif sim_time < 16.0:
            lead_accel = 1.2                   # Accelerate away
        else:
            lead_accel = -5.0                  # Sudden hard braking

        lead_car.update(lead_accel, DT)

        # --- B. Simulated Radar Perception ---
        true_distance = (lead_car.x - lead_car.length) - ego_car.x

        if 0 < true_distance <= radar_range:
            detected = True
            # Add small Gaussian noise to simulate sensor inaccuracy
            noise = np.random.normal(0.0, 0.08)
            measured_dist = max(0.1, true_distance + noise)
            lead_v_measured = lead_car.v
        else:
            detected = False
            measured_dist = float('inf')
            lead_v_measured = 0.0

        # --- C. Controller Decision & State Update ---
        accel_cmd, ttc = acc.compute_control(ego_car, detected, measured_dist, lead_v_measured)
        ego_car.update(accel_cmd, DT)

        # Camera tracks ego vehicle
        camera_x = ego_car.x

        # --- D. Rendering Canvas ---
        screen.fill(COLOR_BG)

        # 1. Road Surface
        pygame.draw.rect(screen, COLOR_ROAD, (0, ROAD_Y - 40, SCREEN_WIDTH, 80))

        # 2. Dashed Lane Markings
        dash_offset = int((ego_car.x * PPM) % 40)
        for rx in range(-40, SCREEN_WIDTH + 40, 40):
            pygame.draw.line(screen, COLOR_MARKING, (rx - dash_offset, ROAD_Y), (rx + 20 - dash_offset, ROAD_Y), 3)

        # 2.5 Passing Scenery (Sense of speed)
        scenery_offset = int((ego_car.x * PPM) % (100 * PPM))
        for bg_x in range(0, SCREEN_WIDTH + int(100*PPM), int(100*PPM)):
            draw_x = bg_x - scenery_offset
            if -50 < draw_x < SCREEN_WIDTH + 50:
                # Streetlight
                pygame.draw.rect(screen, (100, 100, 100), (draw_x, ROAD_Y - 120, 6, 80))
                pygame.draw.line(screen, (100, 100, 100), (draw_x, ROAD_Y - 120), (draw_x + 30, ROAD_Y - 120), 4)
                pygame.draw.circle(screen, (255, 255, 150), (draw_x + 30, ROAD_Y - 116), 5)

        # 3. Radar Beam Cone
        ego_screen_x = to_screen_x(ego_car.x, camera_x)
        radar_reach_px = int(radar_range * PPM)
        radar_surface = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)
        pygame.draw.polygon(radar_surface, COLOR_RADAR, [
            (ego_screen_x + int(ego_car.length * PPM), ROAD_Y),
            (ego_screen_x + radar_reach_px, ROAD_Y - 35),
            (ego_screen_x + radar_reach_px, ROAD_Y + 35)
        ])
        screen.blit(radar_surface, (0, 0))

        # 4. Ego Vehicle (Blue)
        veh_w = int(ego_car.length * PPM)
        veh_h = int(ego_car.width * PPM)
        
        # Wheels
        wheel_c = (30, 30, 30)
        for wx in [ego_screen_x + 6, ego_screen_x + veh_w - 14]:
            pygame.draw.rect(screen, wheel_c, (wx, ROAD_Y - veh_h // 2 - 4, 8, 4))
            pygame.draw.rect(screen, wheel_c, (wx, ROAD_Y + veh_h // 2, 8, 4))
            
        pygame.draw.rect(screen, COLOR_EGO, (ego_screen_x, ROAD_Y - veh_h // 2, veh_w, veh_h), border_radius=4)
        
        # Brake Lights (Ego)
        if ego_car.a < -0.5:
            pygame.draw.rect(screen, (255, 40, 40), (ego_screen_x, ROAD_Y - veh_h // 2 + 2, 4, 4), border_radius=2)
            pygame.draw.rect(screen, (255, 40, 40), (ego_screen_x, ROAD_Y + veh_h // 2 - 6, 4, 4), border_radius=2)

        # 5. Lead Vehicle (Red)
        lead_screen_x = to_screen_x(lead_car.x - lead_car.length, camera_x)
        
        # Wheels
        for wx in [lead_screen_x + 6, lead_screen_x + veh_w - 14]:
            pygame.draw.rect(screen, wheel_c, (wx, ROAD_Y - veh_h // 2 - 4, 8, 4))
            pygame.draw.rect(screen, wheel_c, (wx, ROAD_Y + veh_h // 2, 8, 4))
            
        pygame.draw.rect(screen, COLOR_LEAD, (lead_screen_x, ROAD_Y - veh_h // 2, veh_w, veh_h), border_radius=4)

        # Brake Lights (Lead)
        if lead_car.a < -0.5:
            pygame.draw.rect(screen, (255, 40, 40), (lead_screen_x, ROAD_Y - veh_h // 2 + 2, 4, 4), border_radius=2)
            pygame.draw.rect(screen, (255, 40, 40), (lead_screen_x, ROAD_Y + veh_h // 2 - 6, 4, 4), border_radius=2)

        # --- E. On-Screen HUD Dashboard ---
        hud_box = pygame.Rect(20, 20, 360, 185)
        pygame.draw.rect(screen, (255, 255, 255), hud_box, border_radius=8)
        pygame.draw.rect(screen, (215, 220, 228), hud_box, 2, border_radius=8)

        # Dynamic status colors
        if acc.state == "EMERGENCY BRAKE":
            status_color = (200, 30, 30)
        elif acc.state == "FOLLOWING":
            status_color = (30, 140, 50)
        else:
            status_color = (30, 100, 200)

        ttc_text = f"{ttc:.2f} s" if (ttc is not None and ttc != float('inf')) else "Safe / N/A"

        hud = [
            ("ADAS Telemetry (Longitudinal)", bold_font, COLOR_TEXT),
            (f"Operating Mode: {acc.state}", bold_font, status_color),
            (f"Ego Speed: {ego_car.v * 3.6:.1f} km/h (Accel: {ego_car.a:+.2f} m/s²)", font, COLOR_TEXT),
            (f"Lead Speed: {lead_car.v * 3.6:.1f} km/h", font, COLOR_TEXT),
            (f"Radar Gap: {true_distance:.1f} m", font, COLOR_TEXT),
            (f"Time-to-Collision (TTC): {ttc_text}", font, status_color)
        ]

        for idx, (label, f_obj, col) in enumerate(hud):
            rendered = f_obj.render(label, True, col)
            screen.blit(rendered, (32, 28 + idx * 24))

        pygame.display.flip()
        clock.tick(30)  # Fixed 30 FPS update

        # Check collision threshold
        if true_distance <= 0.0:
            print(f"\n[CRASH DETECTED] Collision occurred at t = {sim_time:.2f} s!")
            running = False

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()