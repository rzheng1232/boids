import numpy as np
import pygame
import math
import random
import pygame
import sys
import colorsys

from multiprocessing import Pool
AGENT_LENGTH = 10
MARGIN = 200
window_width = 1600
window_height = 800
turn_factor = 0.2 # for edge avoidance
avoidance_factor = 0.05 # for avoiding collisions
match_factor = 0.05 # for matching heading/velocities
cohesion_factor = 0.001 # For steering torwards center of mass
mouse_bias = 0#0.005
num_agents = 1000
num_processes = 4
dt = 1
v = 10
alpha = 0.1
min_speed = 5
max_speed = 20
# radius to be influenced by other nearby agents
protected_range = 15
visual_range = 60
mouse_range = 100
class agent:
    def __init__(self, x, y, heading, v):
        self.x = x;
        self.y = y;
        self.theta = heading;
        self.vx = v*math.cos(self.theta);
        self.vy = v*math.sin(self.theta);
        self.color_speed = v;
        self.color = (0, 0, 0);
def step(agents_and_other):
    agents = agents_and_other[0]
    m_x = agents_and_other[1]
    m_y = agents_and_other[2]
    delta_vx = [0.0 for _ in agents]
    delta_vy = [0.0 for _ in agents]
    for i, a in enumerate(agents):
        # collision avoidance + centering + velocity matching
        total_avoid_dx = 0
        total_avoid_dy = 0 
        total_vx = 0
        total_vy = 0
        total_x = 0
        total_y = 0
        num_in_range = 0
        for b in agents:
            dx = a.x-b.x
            dy = a.y-b.y
            d = dx**2 + dy**2
            if (d < visual_range**2 and b is not a):
                num_in_range += 1
                total_vx += b.vx
                total_vy += b.vy
                total_x += b.x
                total_y += b.y
                if (d < protected_range**2):
                    total_avoid_dx += dx
                    total_avoid_dy += dy
        delta_vx[i] += avoidance_factor * total_avoid_dx
        delta_vy[i] += avoidance_factor * total_avoid_dy
        if (num_in_range > 0):
            delta_vx[i] += ((total_vx/num_in_range) - a.vx) * match_factor + ((total_x/num_in_range) - a.x)*cohesion_factor
            delta_vy[i] += ((total_vy/num_in_range) - a.vy) * match_factor + ((total_y/num_in_range) - a.y)*cohesion_factor
        # Bias torwards user mouse
        m_dx = (m_x-a.x)
        m_dy = (m_y-a.y)
        d_mouse = m_dx**2 + m_dy**2
        if (d_mouse < mouse_range**2):
            # d_norm = math.sqrt(d_mouse) / mouse_range
            # weight = 4 * d_norm * (1 - d_norm)
            delta_vx[i] -= mouse_bias * m_dx
            delta_vy[i] -= mouse_bias * m_dy
        # Edge avoidance
        if (a.x < MARGIN):
            delta_vx[i] += turn_factor
        if (a.x > window_width - MARGIN):
            delta_vx[i] -= turn_factor
        if (a.y < MARGIN):
            delta_vy[i] += turn_factor
        if (a.y > window_height - MARGIN):
            delta_vy[i] -= turn_factor
    for i, a in enumerate(agents):
        a.vx += delta_vx[i]
        a.vy += delta_vy[i]
        speed = a.vx**2 + a.vy**2
        a.color_speed = (1-alpha)*a.color_speed + alpha*speed
        if (speed < min_speed**2):
            a.vx = (a.vx / math.sqrt(speed)) * min_speed
            a.vy = (a.vy / math.sqrt(speed)) * min_speed
        if (speed > max_speed**2):
            a.vx = (a.vx / math.sqrt(speed)) * max_speed
            a.vy = (a.vy / math.sqrt(speed)) * max_speed
        a.theta = math.atan2(a.vy, a.vx)%(2 * math.pi)
        a.x += a.vx * dt
        a.y += a.vy * dt
    return agents
    

def main():
    agents = [ agent(random.randrange(0, window_width), random.randrange(0, window_height), random.random()*2*math.pi, v) for i in range(num_agents)]

    pygame.init()
    screen = pygame.display.set_mode((window_width, window_height))
    surface = pygame.Surface((window_width, window_height), pygame.SRCALPHA)
    pygame.display.set_caption("boids")

    # Main loop
    running = True
    clock = pygame.time.Clock()
    pool = Pool(num_processes)
    while running:
        clock.tick(60)
        mouse_x, mouse_y = pygame.mouse.get_pos()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Fill the screen with a color (RGB)
        # screen.fill((0, 0, 0))
        surface.fill((0, 0, 0, 50)) 
        screen.blit(surface, (0, 0))
        agents_chunks = [(chunk, mouse_x, mouse_y) for chunk in np.array_split(agents, num_processes)]
        results = pool.map(step, agents_chunks)
        agents = np.concatenate(results)
        # step(agents, mouse_x, mouse_y)
        pygame.draw.circle(screen, (255, 0, 0), (mouse_x, mouse_y), mouse_range, 1)
        for a in agents:
            # first point drawn in direction of heading
            speed = a.vx**2 + a.vy**2
            color_h =  0.5 + 0.5 * (a.theta) / (2 * math.pi) 
            color_v = 0.3 + 0.7 * min(a.color_speed/max_speed, 1.0) 
            color_s = 1.0        
            new_color = tuple(c * 255 for c in colorsys.hsv_to_rgb(color_h, color_s, color_v))
            a.color = tuple((old + new)/2 for old, new in zip(a.color, new_color))
            pygame.draw.polygon(screen, a.color, [(a.x+AGENT_LENGTH*math.cos(a.theta), a.y+AGENT_LENGTH*math.sin(a.theta)), 
                                                         (a.x-AGENT_LENGTH/3*math.sin(a.theta), a.y+AGENT_LENGTH/3*math.cos(a.theta)), 
                                                         (a.x+AGENT_LENGTH/3*math.sin(a.theta), a.y-AGENT_LENGTH/3*math.cos(a.theta))])
            # pygame.draw.circle(screen, (255, 0, 0), (a.x, a.y), protected_range, 1)
            # pygame.draw.circle(screen, (0, 255, 0), (a.x, a.y), visual_range, 1)
        # Update the display
        pygame.display.flip()

    # Clean up
    pygame.quit()
    pool.close()
    pool.join()
    sys.exit()
if __name__ == "__main__":
    main()