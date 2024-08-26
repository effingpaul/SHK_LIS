import cv2 as cv

channel = 10
cam = cv.VideoCapture(channel)
cam.set(cv.CAP_PROP_FRAME_WIDTH, 320)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, 240)


FPS = 30
fourcc = ('M', 'J', 'P', 'G')
writer = cv.VideoWriter('output.avi', cv.VideoWriter_fourcc(*fourcc), FPS, (320, 240))

while True:
    ret, frame = cam.read()
    if not ret:
        break
    writer.write(frame)
    cv.imshow('frame', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
    