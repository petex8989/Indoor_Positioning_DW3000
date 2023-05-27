import time
import turtle
from turtle import Screen
import socket
from tqdm import tqdm


show_anchor_range = True

hostname = socket.gethostname()
UDP_IP = "192.168.0.100" #socket.gethostbyname(hostname)
print("***Local ip:" + str(UDP_IP) + "***")
UDP_PORT = 80

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
try:
    sock.bind((UDP_IP, UDP_PORT))
except:
    pass

# sock.listen(1)
# data, addr = sock.accept()

meter2pixel = 30
range_offset = 0.9

pos_offset_x = 10*meter2pixel
pos_offset_y = 10*meter2pixel

anchorpos = [[0,0], # Anchor 1 Position
             [0,0], # Anchor 2 Position
             [0,0]] # Anchor 3 Position
def scale_x(x):
    return (x * meter2pixel) - pos_offset_x
def scale_y(y):
    return (y * meter2pixel) - pos_offset_y

def screen_init(width=1200, height=800, t=turtle):
    t.setup(width, height)
    t.tracer(False)
    t.hideturtle()
    t.speed(0)


def turtle_init(t=turtle):
    t.hideturtle()
    t.speed(0)


def draw_line(x0, y0, x1, y1, color="black", t=turtle):
    t.pencolor(color)

    t.up()
    t.goto(x0, y0)
    t.down()
    t.goto(x1, y1)
    t.up()


def draw_fastU(x, y, length, color="black", t=turtle):
    draw_line(x, y, x, y + length, color, t)


def draw_fastV(x, y, length, color="black", t=turtle):
    draw_line(x, y, x + length, y, color, t)


def draw_cycle(x, y, r, color="black", t=turtle):
    t.pencolor(color)

    t.up()
    t.goto(x, y - r)
    t.setheading(0)
    t.down()
    t.circle(r)
    t.up()


def fill_cycle(x, y, r, color="black", t=turtle):
    t.up()
    t.goto(x, y)
    t.down()
    t.dot(r, color)
    t.up()

def draw_circle(x, y, r, color = "black", t = turtle):
    t.pencolor(color)
    t.up()
    t.goto(x,y-r)
    t.down()
    t.circle(r)
    t.up()
    t.pencolor("black")
    # radius = 100

def write_txt(x, y, txt, color="black", t=turtle, f=('Arial', 12, 'normal')):
    # x_pos = x*meter2pixel - pos_offset_x
    # y_pos = y*meter2pixel - pos_offset_y
    t.pencolor(color)
    t.up()
    t.goto(x, y)
    t.down()
    t.write(txt, move=False, align='left', font=f)
    t.up()


def draw_rect(x, y, w, h, color="black", t=turtle):
    t.pencolor(color)

    t.up()
    t.goto(x, y)
    t.down()
    t.goto(x + w, y)
    t.goto(x + w, y + h)
    t.goto(x, y + h)
    t.goto(x, y)
    t.up()

def draw_poly(vert, color="black", t=turtle):
    n = vert.get_num_points()
    t.pencolor(color)

    t.up()
    t.width(5)
    t.goto(scale_x(vert.points[0].x), scale_y(vert.points[0].y))
    t.down()
    for i in range (1,n):
        t.goto(scale_x(vert.points[i].x), scale_y(vert.points[i].y))
    t.goto(scale_x(vert.points[0].x), scale_y(vert.points[0].y))
    t.width(1)
    t.up()



def fill_rect(x, y, w, h, color=("black", "black"), t=turtle):
    t.begin_fill()
    draw_rect(x, y, w, h, color, t)
    t.end_fill()
    pass


def clean(t=turtle):
    t.clear()


def draw_ui(t):
    write_txt(-300, 250, "UWB Positon", "black",  t, f=('Arial', 32, 'normal'))
    # fill_rect(-400, 200, 800, 40, "black", t)
    # write_txt(-50, 205, "WALL", "yellow",  t, f=('Arial', 24, 'normal'))


def draw_uwb_anchor(x, y, txt, range, t):
    r = 20
    pos_x = (x * meter2pixel) - pos_offset_x
    pos_y = (y * meter2pixel) - pos_offset_y
    scaled_range = range * meter2pixel
    fill_cycle(pos_x, pos_y, r, "green", t)
    if (show_anchor_range):
        draw_circle(pos_x, pos_y, scaled_range, "green", t)
    write_txt(pos_x + r, pos_y, txt + ": " + str(range) + " ft", "black",  t, f=('Arial', 16, 'normal'))
    # write_txt(pos_x + r, pos_y, txt, "black",  t, f=('Arial', 16, 'normal'))


def draw_uwb_tag(x, y, r, txt, t):
    pos_x = (x * meter2pixel) - pos_offset_x
    pos_y = (y * meter2pixel) - pos_offset_y
    # r = 20
    scaled_r = (r * meter2pixel)
    fill_cycle(pos_x, pos_y, 20, "blue", t)
    draw_circle(pos_x, pos_y, scaled_r, "blue", t)
    write_txt(pos_x + 20, pos_y, txt + ": (" + str(x) + "," + str(y) + ")",
              "black",  t, f=('Arial', 16, 'normal'))


def read_data():
    keepRecieving = True
    sock.setblocking(False)
    # clear buffer of data to get most recent data
    while(keepRecieving):
        try:
            d, a = sock.recvfrom(1024)
        except:
            keepRecieving = False
    sock.setblocking(True)
    data, addr = sock.recvfrom(1024)
    # line = data.recv(1024).decode('UTF-8')
    # line = data.makefile().readline()
    uwb_list = data.decode().split(',')
    recv_data = True
    try:
        uwb_list = [eval(i) for i in uwb_list]
    except:
        # print("connection error")
        recv_data = False
    k = 6
    for i in range(3):
        for j in range(2):
            anchorpos[i][j] = uwb_list[k]
            k += 1
    return uwb_list, recv_data

def get_coords(n = 1):
    count = 0
    x = 0
    y = 0
    with tqdm(total = n) as pbar:
        while (count < n):
            data, recv_data = read_data()
            if (recv_data):
                x += data[0]
                y += data[1]
                count += 1
                pbar.update(1)
    x /= float(n)
    y /= float(n)

    # x = input("enter x coord: ")
    # y = input("enter y coord: ")
    return x,y


# def tag_pos(a, b, c):
#     # p = (a + b + c) / 2.0
#     # s = cmath.sqrt(p * (p - a) * (p - b) * (p - c))
#     # y = 2.0 * s / c
#     # x = cmath.sqrt(b * b - y * y)
#     cos_a = (b * b + c*c - a * a) / (2 * b * c)
#     x = b * cos_a
#     y = b * cmath.sqrt(1 - cos_a * cos_a)

#     return round(x.real, 1), round(y.real, 1)
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
 
class line:
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2

class Polygon:
    def __init__(self, points):
        self.points = []
        try:
            for i in range(len(points)):
                self.points.append(points[i])
        except:
            self.points.append(points)
    def add_point(self, point):
        self.points.append(point)
    def get_num_points(self):
        return len(self.points)

class Room:
    def __init__(self, name, id, poly):
        self.name = name
        self.id = id
        self.poly = poly
 
def onLine(l1, p):
    # Check whether p is on the line or not
    if (
        p.x <= max(l1.p1.x, l1.p2.x)
        and p.x <= min(l1.p1.x, l1.p2.x)
        and (p.y <= max(l1.p1.y, l1.p2.y) and p.y <= min(l1.p1.y, l1.p2.y))
    ):
        return True
    return False
 
def direction(a, b, c):
    val = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y)
    if val == 0:
        # Collinear
        return 0
    elif val < 0:
        # Anti-clockwise direction
        return 2
    # Clockwise direction
    return 1
 
def isIntersect(l1, l2):
    # Four direction for two lines and points of other line
    dir1 = direction(l1.p1, l1.p2, l2.p1)
    dir2 = direction(l1.p1, l1.p2, l2.p2)
    dir3 = direction(l2.p1, l2.p2, l1.p1)
    dir4 = direction(l2.p1, l2.p2, l1.p2)
 
    # When intersecting
    if dir1 != dir2 and dir3 != dir4:
        return True
 
    # When p2 of line2 are on the line1
    if dir1 == 0 and onLine(l1, l2.p1):
        return True
 
    # When p1 of line2 are on the line1
    if dir2 == 0 and onLine(l1, l2.p2):
        return True
 
    # When p2 of line1 are on the line2
    if dir3 == 0 and onLine(l2, l1.p1):
        return True
 
    # When p1 of line1 are on the line2
    if dir4 == 0 and onLine(l2, l1.p2):
        return True
 
    return False
 
def checkInside(poly, p):
    n = poly.get_num_points()
    # When polygon has less than 3 edge, it is not polygon
    if n < 3:
        return False
 
    # Create a point at infinity, y is same as point p
    exline = line(p, Point(9999, p.y))
    count = 0
    i = 0
    while True:
        # Forming a line from two consecutive points of poly
        side = line(poly.points[i], poly.points[(i + 1) % n])
        if isIntersect(side, exline):
            # If side is intersects ex
            if (direction(side.p1, p, side.p2) == 0):
                return onLine(side, p)
            count += 1
         
        i = (i + 1) % n
        if i == 0:
            break
    # When count is odd
    return count & 1

def uwb_range_offset(uwb_range):

    temp = uwb_range
    return temp

def input_thread(a_list):
    input()
    a_list.append(True)

t_ui = turtle.Turtle()
t_a1 = turtle.Turtle()
t_a2 = turtle.Turtle()
t_a3 = turtle.Turtle()
t_a4 = turtle.Turtle()
t_rooms = turtle.Turtle()
turtle_init(t_ui)
turtle_init(t_a1)
turtle_init(t_a2)
turtle_init(t_a3)
turtle_init(t_a4)
turtle_init(t_rooms)

screen = Screen()
screen.tracer(False)

rooms = []
curr_room = 0
def draw_room():
    while 1:
        list, recv_data = read_data()
        if (recv_data):

            x = list[0]
            y = list[1]
            r = list[2]
            anchor_range = [list[3],list[4],list[5]]

            clean(t_rooms)
            for i in range(len(rooms)):
                color = "black"
                if (checkInside(rooms[i].poly, Point(x,y))):
                    curr_room = rooms[i].id
                    color = "red"
                draw_poly(rooms[i].poly, color, t_rooms)
            clean(t_a1)
            draw_uwb_anchor(anchorpos[0][0], anchorpos[0][1], "Anchor 1 (" + str(anchorpos[0][0]) + "," + str(anchorpos[0][1]) + ")", anchor_range[0], t_a1)

            clean(t_a2)
            draw_uwb_anchor(anchorpos[1][0], anchorpos[1][1], "Anchor 2 (" + str(anchorpos[1][0]) + "," + str(anchorpos[1][1]) + ")", anchor_range[1], t_a2)
                
            clean(t_a3)
            draw_uwb_anchor(anchorpos[2][0], anchorpos[2][1], "Anchor 3 (" + str(anchorpos[2][0]) + "," + str(anchorpos[2][1]) + ")", anchor_range[2], t_a3)

            clean(t_a4)
            draw_uwb_tag(x, y, r, "TAG", t_a4)
            screen.update()
        else:
            write_txt(0 - pos_offset_x,0 - pos_offset_y, "Connection Error!", "red",  t_ui, f=('Arial', 32, 'bold'))
            t_ui.clear()

    return

def add_room():
    print('You are now in home setup.\n\
Intructions:\n\
    1. Go to each corner of the room, and record the coordinates of each corner.\n\
    2. Make sure to record the corners sequentially, by following the walls of the room\n\
    3. Try to keep all the readings at the same vertical height\n\
    4. Enter "q" when you are done with the room')
    time.sleep(2)
    userin1 = input('Options:\n\
    1. Add Room\n\
    2. Exit\n\
Enter your selection #: ')
    while (userin1 != '1' and userin1 != '2'):
        userin1 = input('Invalid selection. Please re-enter.\n\
Options:\n\
    1. Add Room\n\
    2. Exit\n\
Enter selection #: ')
    while (userin1 != '2'):
        print("You are now entering room coordinates for your home. Please place your tag in the first corner.")
        userin2 = input('Enter "1" to record first coordinates: ')
        while (userin2 != '1'):
            userin2 = input('Invalid Selection. Enter "1" to record first coordinate: ')
        # Record first coordinates
        n = 30
        x, y = get_coords(n)
        verticies = Polygon(Point(x,y))

        print("First Coordinate Recorded")
        count = 1
        while(True):
            userin2 = input('Enter "1" to record next coordinate or "2" to exit and finalize room: ')
            while (userin2 != '1' and userin2 != '2'):
                userin2 = input('Invalid Selection. Enter "1" to record next coordinate or "2" to exit and finalize room: ')
            while (count < 3 and userin2 != '1'):
                if (userin2 != '1' and userin2 != '2'):
                    userin2 = input('Invalid Selection. Enter "1" to record next coordinate or "2" to exit and finalize room: ')
                else:
                    userin2 = input('Not enough corners entered to finalize room. Go to the next corner and enter "1" to record coordinates: ')
            if (userin2 == '2'):
                break
            # Record Coordinates
            x, y = get_coords(n)
            verticies.add_point(Point(x,y))
            print("Coordinate Recorded")
            count = count + 1
        
        name = input("Enter the name of the room: ")
        # Save Room Coordinates
        room = Room(name, len(rooms), verticies)
        rooms.append(room)

        print("Room Saved")
        # Dipslay options again
        userin1 = input('Options:\n\
    1. Add another Room\n\
    2. Exit\n\
Enter selection #: ')
        while (userin1 != '1' and userin1 != '2'):
            userin1 = input('Invalid selection. Please re-enter.\n\
Options:\n\
    1. Add Room\n\
    2. Exit\n\
Enter selection #: ')
    print("Returning to Main Menu")
    time.sleep(0.3)
    return

def main():
    while True:
        # display main menu options
        userin = input('Options:\n\
    1. Indoor Positioning\n\
    2. Add Room\n\
    3. Exit\n\
Enter selection #: ')
        # validate entered option
        while (userin not in ['1', '2', '3']):
            print('Invalid selection. Please re-enter')
            userin = input('Options:\n\
    1. Indoor Positioning\n\
    2. Add Room\n\
    3. Exit\n\
Enter selection #: ')
        # go to selected option
        if (userin == '1'):
            draw_room()
        elif(userin == '2'):
            add_room()
        elif(userin == '3'):
            return
    turtle.mainloop()


if __name__ == '__main__':
    main()