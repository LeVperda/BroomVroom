import cv2
import math

# open image, define steering line
img = cv2.imread(r"/home/levperda/robotics_project/robotics_project/python/test_photo/2024-04-19-122414.jpg")
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
height, width, colorlayers = img.shape
img.shape

# define needed variables
horizontal_line = int(height * (3/4))
blue = 2
green = 1
red = 0
middle = int(width / 2)
car_line = height * (13/16)
gap = car_line - horizontal_line

# find left and right sides of the track
def get_track_sides():
    for x in range(int(width/2), 0, -1):
      if img[horizontal_line + 5, x][blue] >= 190 or img[horizontal_line + 5, x][green] >= 190 or img[horizontal_line + 5, x][red] >= 190:
        l_line = x
        break
      continue

    for x in range(int(width/2), width):
      if img[horizontal_line + 5, x][blue] >= 190 or img[horizontal_line + 5, x][green] >= 190 or img[horizontal_line + 5, x][red] >= 190:
        r_line = x
        break
      continue

    return l_line, r_line
  
# get the turning angle
def get_turn():
  l_line, r_line = get_track_sides()
  mid_point = l_line + (r_line - l_line) / 2
  
  if mid_point > middle:
     distance = mid_point - middle
     angle_rad = math.atan(distance/gap)
     angle = math.degrees(angle_rad)
     return 90 - angle
  
  elif mid_point < middle:
     distance = middle - mid_point
     angle_rad = math.atan(distance/gap)
     angle = math.degrees(angle_rad)
     return 90 + angle
  
  else:
     return 90
  

print(get_turn())