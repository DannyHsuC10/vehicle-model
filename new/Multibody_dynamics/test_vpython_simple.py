from vpython import canvas, sphere, vector

print('Starting VPython simple test...')
scene = canvas(title='VPython Simple Test', width=800, height=600)
print('Canvas created:', scene)
ball = sphere(pos=vector(0, 0, 0), radius=1, color=vector(0.2, 0.5, 0.8))
print('Sphere created:', ball)
print('VPython simple test finished. If a window opened, it should now be visible.')
