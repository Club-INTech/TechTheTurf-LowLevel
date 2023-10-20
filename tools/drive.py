# Simple pygame program

# Import and initialize the pygame library
import pygame,serial
pygame.init()

# Set up the drawing window
screen = pygame.display.set_mode([500, 500])

dst = 0
angle = 0

clock = pygame.time.Clock()

ser = serial.Serial("/dev/ttyACM0", 115200)

# Run until the user asks to quit
running = True
nb = 0
while running:
	dt = float(clock.tick(60.0))/1000.0

	# Did the user click the window close button?
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			running = False
	keys = pygame.key.get_pressed()
	left = keys[pygame.K_LEFT]
	right = keys[pygame.K_RIGHT]
	up = keys[pygame.K_UP]
	down = keys[pygame.K_DOWN]

	if up:
		dst += 1000.0*dt
	if down:
		dst -= 1000.0*dt
	if left:
		angle += 4.0*dt
	if right:
		angle -= 4.0*dt

	nb += 1
	if nb%20 == 0:
		dat = f"s{dst:.2f} {angle:.2f}\n".encode("utf-8")
		print(dst,angle,dat)
		ser.write(dat)
		ser.flush()
	#print(ser.realine())

	# Fill the background with white
	screen.fill((255, 255, 255))

	# Draw a solid blue circle in the center
	pygame.draw.circle(screen, (0, 0, 255), (250, 250), 75)

	# Flip the display
	pygame.display.flip()

# Done! Time to quit.
pygame.quit()